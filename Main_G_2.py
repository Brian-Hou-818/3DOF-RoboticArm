# robot_control_mdh_v2.py
# Rewritten to use the MDH you provided:
# mDH = np.array([[0, 0, 67.5, 180],
#                 [-22.5, 90, 0, 0],
#                 [113.1, 0, 0, 0],
#                 [55, 0, 0, 180]])
#
# Interpretation used here:
# - Row0: base (a=0, alpha=0, d=67.5) -> base_height = 67.5 mm
# - Row1: shoulder offset (a=-22.5, alpha=90) -> a_offset = -22.5 mm
# - Row2: link1 (a=113.1) -> l1 = 113.1 mm
# - Row3: link2 / EE (a=55) -> l2 = 55.0 mm
#
# NOTE: If you intended different semantics for those rows, change the
# assignments near "---- geometry ----" accordingly.

from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
import math
import numpy as np

factory = PiGPIOFactory()

# ---- MDH table supplied by user ----
mDH = np.array([
    [0.0,     0.0,  67.5, 180.0],
    [-22.5,  90.0,   0.0,   0.0],
    [113.1,   0.0,   0.0,   0.0],
    [55.0,    0.0,   0.0, 180.0]
])

# ---- geometry (interpreted) ----
base_height = float(mDH[0][2])    # row0.d -> base height (mm)
a_offset    = float(mDH[1][0])    # row1.a -> horizontal offset from base to shoulder (mm)
l1          = float(mDH[2][0])    # row2.a -> shoulder -> elbow (mm)
l2          = float(mDH[3][0])    # row3.a -> elbow -> tip / EE extension (mm)

# ---- Servo config (adjust to match your hardware) ----
servoCfg = {
    'joint1': {'neutral': 90.0, 'dir':  1.0, 'travelHalf': 90.0},  # base (deg)
    'joint2': {'neutral': 90.0, 'dir':  1.0, 'travelHalf': 90.0},  # shoulder (deg)
    'joint3': {'neutral': 90.0, 'dir':  1.0, 'travelHalf': 90.0}   # elbow (deg)
}

# ---- create servo objects (change pins if needed) ----
servo1 = Servo(13, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
servo2 = Servo(14, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
servo3 = Servo(15, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

print("\n<---- Servos initialized. Verify wiring and calibration BEFORE moving ---->\n")

# ---- helpers ----
def deg(rad):
    return rad * 180.0 / math.pi

def rad(deg):
    return deg * math.pi / 180.0

def servoConvert(angle_deg, neutral, direction, travelHalfDeg):
    """
    Map angle (deg) -> servo.value in [-1, 1] using neutral and travelHalfDeg.
    Clamp to allowed travel range.
    """
    diff = angle_deg - neutral
    diff = max(-travelHalfDeg, min(travelHalfDeg, diff))
    norm = diff / travelHalfDeg
    return direction * norm

def reachable(r):
    return abs(l1 - l2) <= r <= (l1 + l2)

# IK for planar 2-link (shoulder+elbow) where the shoulder origin is offset horizontally by a_offset.
# Input xp,zp are coordinates in the shoulder-plane coordinate system (x forward from shoulder, z up)
def twoJointIK(xp, zp):
    r = math.hypot(xp, zp)
    if not reachable(r):
        return []

    # law of cosines for elbow angle between l1 and l2
    cos_q3 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_q3 = max(-1.0, min(1.0, cos_q3))
    q3 = math.acos(cos_q3)

    # shoulder angle computation
    alpha = math.atan2(zp, xp)
    cos_beta = (l1**2 + r**2 - l2**2) / (2 * l1 * r)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)

    # two standard solutions
    q2_1 = alpha - beta
    q3_1 = q3
    q2_2 = alpha + beta
    q3_2 = -q3

    return [(q2_1, q3_1), (q2_2, q3_2)]

# Forward kinematics given theta_base (rad), q2 (rad shoulder), q3 (rad elbow).
# Returns global (X, Y, Z) of the tool tip, accounting for a_offset and base rotation.
def forward_kinematics(theta_base, q2, q3):
    # shoulder origin in base frame:
    # shoulder_x_plane (along base X) = a_offset
    # shoulder_z = base_height
    sx_plane = a_offset
    sz = base_height

    # elbow pos in shoulder-plane coordinates (x forward from shoulder, z up)
    ex_plane = sx_plane + l1 * math.cos(q2)
    ez = sz + l1 * math.sin(q2)

    # wrist (end of link2) in plane
    wx_plane = ex_plane + l2 * math.cos(q2 + q3)
    wz = ez + l2 * math.sin(q2 + q3)

    # tip in plane = wx_plane, wz; rotate around base z by theta_base to get global X,Y
    X = math.cos(theta_base) * wx_plane - math.sin(theta_base) * 0.0
    Y = math.sin(theta_base) * wx_plane + math.cos(theta_base) * 0.0
    Z = wz

    return (X, Y, Z)

# ---- Main interactive loop ----
print("Enter target coordinates in mm (X, Y, Z).")
print(f"Using MDH interpreted as -> base_height={base_height} mm, a_offset={a_offset} mm, l1={l1} mm, l2={l2} mm")
print("Press Ctrl-C to exit.\n")

try:
    while True:
        try:
            move_X = float(input("X (mm): "))
            move_Y = float(input("Y (mm): "))
            move_Z = float(input("Z (mm): "))
        except ValueError:
            print("Invalid input. Try again.")
            continue

        # base rotation (rad)
        theta1 = math.atan2(move_Y, move_X)
        baseDeg = deg(theta1)

        # horizontal distance from base to target in XY plane
        rho = math.hypot(move_X, move_Y)

        # convert target into shoulder-plane coordinates:
        # xp = distance from shoulder origin along shoulder x-axis to the projection of the tip
        # shoulder origin sits at a_offset from base along base X, and the link lengths are measured from shoulder
        xp = rho - a_offset      # subtract shoulder offset so xp is in shoulder frame
        zp = move_Z - base_height

        # quick feas check
        if xp < 0 and abs(xp) > 1e-6:
            print(f"Target behind shoulder origin (xp={xp:.2f} mm). Marked unreachable.\n")
            continue

        sols = twoJointIK(xp, zp)
        if not sols:
            print("Target unreachable by planar 2-link chain (out of reach).\n")
            continue

        # pick primary solution (elbow-down)
        q2, q3 = sols[0]
        shoulderDeg = deg(q2)
        elbowDeg = deg(q3)

        print(f"Computed angles (deg): Base {baseDeg:.2f}, Shoulder {shoulderDeg:.2f}, Elbow {elbowDeg:.2f}")

        # FK verification
        fkX, fkY, fkZ = forward_kinematics(theta1, q2, q3)
        err = math.hypot(fkX - move_X, fkY - move_Y, fkZ - move_Z)
        print(f"FK tip: X={fkX:.2f}, Y={fkY:.2f}, Z={fkZ:.2f}  -> position error {err:.2f} mm")

        # convert to servo values
        s1 = servoConvert(baseDeg, servoCfg['joint1']['neutral'], servoCfg['joint1']['dir'], servoCfg['joint1']['travelHalf'])
        s2 = servoConvert(shoulderDeg, servoCfg['joint2']['neutral'], servoCfg['joint2']['dir'], servoCfg['joint2']['travelHalf'])
        s3 = servoConvert(elbowDeg, servoCfg['joint3']['neutral'], servoCfg['joint3']['dir'], servoCfg['joint3']['travelHalf'])

        # ramp movement helper
        def ramp_servo(servo_obj, target_val, steps=12, delay=0.02):
            start = servo_obj.value or 0.0
            for i in range(1, steps+1):
                v = start + (target_val - start) * (i / steps)
                servo_obj.value = v
                sleep(delay)

        print("Moving servos...")
        ramp_servo(servo1, s1)
        ramp_servo(servo2, s2)
        ramp_servo(servo3, s3)
        print("Move complete.\n")

except KeyboardInterrupt:
    print("\nStopping and setting servos to neutral.")
    servo1.value = servoConvert(servoCfg['joint1']['neutral'], servoCfg['joint1']['neutral'], servoCfg['joint1']['dir'], servoCfg['joint1']['travelHalf'])
    servo2.value = servoConvert(servoCfg['joint2']['neutral'], servoCfg['joint2']['neutral'], servoCfg['joint2']['dir'], servoCfg['joint2']['travelHalf'])
    servo3.value = servoConvert(servoCfg['joint3']['neutral'], servoCfg['joint3']['neutral'], servoCfg['joint3']['dir'], servoCfg['joint3']['travelHalf'])
    print("Done. Goodbye.")
