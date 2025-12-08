# ---------- IK + control for 3-DOF turret + 2-pitch arm ----------
import math
import numpy as np
import time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# ---- Robot geometry (from your message) ----
base_h = 30.0          # mm, joint1 axis above ground
a_offset = -22.5       # mm offset from joint1->joint2 along X (negative means back)
l1 = 67.5              # mm, first link length (shoulder -> elbow)
l2 = 113.1             # mm, second link length (elbow -> wrist)

# ---- Servo calibration (tweak these per servo) ----
# For each controlled joint (servo1 = base yaw, servo2 = shoulder, servo3 = elbow)
# neutral_deg is the angle which should map to servo.value = 0
# direction is 1 or -1 to flip sign if servo orientation is reversed
# travel_half_deg is the half-range of the servo's usable motion (SG90 ≈ 90° each side)
factory = PiGPIOFactory()
servo1 = Servo(13, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo2 = Servo(14, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo3 = Servo(15, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo4 = Servo(16, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo5 = Servo(17, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo_cfg = {
    'base'     : {'servo': servo1, 'neutral_deg': 0.0,   'direction': 1,  'travel_half_deg': 90.0},
    'shoulder' : {'servo': servo2, 'neutral_deg': 0.0,   'direction': -1, 'travel_half_deg': 90.0},
    'elbow'    : {'servo': servo3, 'neutral_deg': 0.0,   'direction': 1,  'travel_half_deg': 90.0},
    'clawLeft' : {'servo': servo4, 'enutral_deg': -0.01, 'direction': 1,  'trave;_half_deg': 90.0},
    'clawRight': {'servo': servo5, 'enutral_deg': 0.01,  'direction': -1, 'trave;_half_deg': 90.0},
}
# Adjust neutral_deg so that that physical servo position corresponds to the desired 0° logical angle.

def deg(rad): return rad * 180.0 / math.pi
def rad(deg): return deg * math.pi / 180.0

def to_servo_value(angle_deg, neutral_deg, direction, travel_half_deg):
    # relative to neutral
    rel = angle_deg - neutral_deg
    # clamp to travel
    max_rel = travel_half_deg
    if rel > max_rel: rel = max_rel
    if rel < -max_rel: rel = -max_rel
    # map to -1..1
    return direction * (rel / travel_half_deg)

def reachable_planar(r):
    """Check planar reachability ignoring numerical tolerance."""
    return abs(l1 - l2) <= r <= (l1 + l2)

def solve_planar_2link(xp, zp):
    """
    Solve planar 2-link (links along local X and Z) for angles theta2, theta3 (radians)
    xp: forward distance in rotated plane (mm)
    zp: vertical distance above joint2 axis (mm)
    returns list of (theta2, theta3) solutions (radians) - may return empty
    Conventions:
      - theta2 measured from local X axis (positive lifts Z)
      - theta3 is joint angle relative to link1 (so the total pitch at link2 is theta2 + theta3)
    """
    r = math.hypot(xp, zp)
    if not reachable_planar(r):
        return []  # unreachable
    # clamp cos to [-1,1]
    cos_q3 = (r*r - l1*l1 - l2*l2) / (2 * l1 * l2)
    cos_q3 = max(-1.0, min(1.0, cos_q3))
    q3_a = math.acos(cos_q3)      # elbow-down
    q3_b = -q3_a                  # elbow-up

    sols = []
    for q3 in (q3_a, q3_b):
        k1 = l1 + l2 * math.cos(q3)
        k2 = l2 * math.sin(q3)
        q2 = math.atan2(zp, xp) - math.atan2(k2, k1)
        sols.append((q2, q3))
    return sols

# ---- Main interactive loop (replaces your sympy part) ----
while True:
    try:
        move_X = float(input("X (mm): "))
        move_Y = float(input("Y (mm): "))
        move_Z = float(input("Z (mm): ")) + 3.0   # keep your +3 offset if needed
    except ValueError:
        print("Invalid input, try again.")
        continue

    # Step 1: yaw (theta1) to point toward XY target
    theta1 = math.atan2(move_Y, move_X)   # radians, turret yaw
    # planar distance from base (before accounting offsets)
    rho = math.hypot(move_X, move_Y)

    # Step 2: compute coordinates in the arm's pitch plane (after removing yaw)
    # Remove the fixed offset 'a_offset' along the forward direction.
    # xp is forward distance from joint2 axis to wrist projection
    xp = rho - a_offset   # note: if a_offset is negative this adds distance
    # vertical distance from joint2 axis to target
    zp = move_Z - base_h

    print(f"rho={rho:.2f} xp={xp:.2f} zp={zp:.2f}")

    # Step 3: solve planar 2-link for theta2 (shoulder) and theta3 (elbow)
    planar_sols = solve_planar_2link(xp, zp)
    if not planar_sols:
        print("Target unreachable in planar cross-section (too far/too close).")
        continue

    # choose a solution strategy: prefer elbow-down (first solution)
    theta2, theta3 = planar_sols[0]   # radians
    # You can choose planar_sols[1] for the alternate elbow pose

    # Convert to degrees for printing and servo mapping
    deg1 = deg(theta1)
    deg2 = deg(theta2)
    deg3 = deg(theta3)

    print(f"IK solutions (deg): yaw={deg1:.2f}, shoulder={deg2:.2f}, elbow={deg3:.2f}")

    # Map to servo values using calibration
    sv_base = to_servo_value(deg1, servo_cfg['base']['neutral_deg'],
                             servo_cfg['base']['direction'], servo_cfg['base']['travel_half_deg'])
    sv_shoulder = to_servo_value(deg2, servo_cfg['shoulder']['neutral_deg'],
                                 servo_cfg['shoulder']['direction'], servo_cfg['shoulder']['travel_half_deg'])
    sv_elbow = to_servo_value(deg3, servo_cfg['elbow']['neutral_deg'],
                              servo_cfg['elbow']['direction'], servo_cfg['elbow']['travel_half_deg'])

    # Print values and move servos
    print(f"servo values: base={sv_base:.3f}, shoulder={sv_shoulder:.3f}, elbow={sv_elbow:.3f}")

    servo1.value = sv_base
    servo2.value = sv_shoulder
    servo3.value = sv_elbow

    # small delay to avoid flooding the servo bus
    time.sleep(0.05)
