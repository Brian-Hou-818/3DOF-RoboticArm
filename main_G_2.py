from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
import math

factory = PiGPIOFactory()

# ---- MDH Parameters ----
theta1 = 0
theta2 = 0
theta3 = 0

mDH = np.array([
    [-22.5, 90, 30, theta1],
    [67.5, 0, 0, theta2],
    [113.1, 0, 0, theta3],
    [55, 90, 0, 0]])

# ---- Robot Geometry ----
baseH = mDH[0][2]
aOffset = mDH[0][0]
l1 = mDH[1][0]
l2 = mDH[2][0]

# ---- Servo Config ----
servoCfg = {
    'joint1': {'neutral': 90, 'dir': 1, 'travelHalf': 90},
    'joint2': {'neutral': 90, 'dir': 1, 'travelHalf': 90},
    'joint3': {'neutral': 90, 'dir': -1, 'travelHalf': 90},
    'clawLeft': {'neutral': 90, 'dir': 1, 'travelHalf': 60},
    'clawRight': {'neutral': 90, 'dir': -1, 'travelHalf': 60}
}

servo1 = Servo(13, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
servo2 = Servo(14, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
servo3 = Servo(15, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

print("\n<---- Servo Calibration Complete ---->\n")

# --- Servo conversion ---
def servoConvert(angle, neutral, direction, travelHalfDeg):
    diff = angle - neutral
    diff = max(-travelHalfDeg, min(travelHalfDeg, diff))
    norm = diff / travelHalfDeg
    return direction * norm

def deg(rad):
    return rad * 180 / math.pi

def reachable(r):
    return l2 - l1 <= r <= l1 + l2

# --- 2-joint planar IK ---
def twoJointIK(xp, zp):
    r = math.hypot(xp, zp)
    if not reachable(r):
        return []

    cosQ3 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cosQ3 = max(-1.0, min(1.0, cosQ3))
    q3 = math.acos(cosQ3)

    alpha = math.atan2(zp, xp)
    beta = math.acos((l1**2 + r**2 - l2**2) / (2 * l1 * r))

    q2 = alpha - beta
    q2_alt = alpha + beta
    q3_alt = -q3

    return [(q2, q3), (q2_alt, q3_alt)]

# --- Morse to letter ---
def morseToLetter(morse):
    morse_dict = {
        '._': 'A', '_...': 'B', '_._.': 'C', '_..': 'D', '.': 'E',
        '.._.': 'F', '__.': 'G', '....': 'H', '..': 'I', '.___': 'J',
        '_._': 'K', '._..': 'L', '__': 'M', '_.': 'N', '___': 'O',
        '.__.': 'P', '__._': 'Q', '._.': 'R', '...': 'S', '_': 'T',
        '.._': 'U', '..._': 'V', '.__': 'W', '_.._': 'X', '_.__': 'Y',
        '__..': 'Z', '_____': '0', '.____': '1', '..___': '2', '...__': '3',
        '...._': '4', '.....': '5', '_....': '6', '__...': '7', '___..': '8',
        '____.': '9',
    }
    return str(morse_dict[morse])

# --- Letter coordinates ---
def location(letter):
    floor = 0  # z-coordinate
    # Example half-circle layout (3 rows x 12 columns)
    # Adjust spacing as needed
    location_dict = {
        'A': [-60, 40, floor], 'B': [-50, 40, floor], 'C': [-40, 40, floor], 'D': [-30, 40, floor],
        'E': [-20, 40, floor], 'F': [-10, 40, floor], 'G': [0, 40, floor], 'H': [10, 40, floor],
        'I': [20, 40, floor], 'J': [30, 40, floor], 'K': [40, 40, floor], 'L': [50, 40, floor],
        'M': [-60, 20, floor], 'N': [-50, 20, floor], 'O': [-40, 20, floor], 'P': [-30, 20, floor],
        'Q': [-20, 20, floor], 'R': [-10, 20, floor], 'S': [0, 20, floor], 'T': [10, 20, floor],
        'U': [20, 20, floor], 'V': [30, 20, floor], 'W': [40, 20, floor], 'X': [50, 20, floor],
        'Y': [-60, 0, floor], 'Z': [-50, 0, floor],
        '1': [-40, 0, floor], '2': [-30, 0, floor], '3': [-20, 0, floor], '4': [-10, 0, floor],
        '5': [0, 0, floor], '6': [10, 0, floor], '7': [20, 0, floor], '8': [30, 0, floor],
        '9': [40, 0, floor], '10': [50, 0, floor]
    }
    return location_dict[letter]

# --- Shoulder calibration ---
shoulderOffset = 90  # adjust so q2=0 (IK forward) points forward
shoulderDir = -1     # because servo moves downward when value increases

# --- Main loop ---
while True:
    inputLetter = str(input("Morse Code: "))
    move_X = float(location(morseToLetter(inputLetter))[0])
    move_Y = float(location(morseToLetter(inputLetter))[1])
    move_Z = float(location(morseToLetter(inputLetter))[2])
    print("Target coords:", move_X, move_Y, move_Z)

    # Base rotation
    theta1 = math.atan2(move_Y, move_X)
    baseDeg = deg(theta1)

    rho = math.hypot(move_X, move_Y)
    xp = rho - aOffset
    zp = move_Z - baseH

    sols = twoJointIK(xp, zp)
    if not sols:
        print("Target unreachable.\n")
        continue

    t2, t3 = sols[0]
    shoulderDeg = shoulderDir * deg(t2) + shoulderOffset
    elbowDeg = deg(t3)

    # --- Move servos ---
    servo1.value = servoConvert(baseDeg, servoCfg['joint1']['neutral'], servoCfg['joint1']['dir'], servoCfg['joint1']['travelHalf'])
    sleep(0.2)
    servo2.value = servoConvert(shoulderDeg, servoCfg['joint2']['neutral'], servoCfg['joint2']['dir'], servoCfg['joint2']['travelHalf'])
    sleep(0.2)
    servo3.value = servoConvert(elbowDeg, servoCfg['joint3']['neutral'], servoCfg['joint3']['dir'], servoCfg['joint3']['travelHalf'])
    sleep(0.2)

    print(f"Base: {baseDeg:.2f}, Shoulder: {shoulderDeg:.2f}, Elbow: {elbowDeg:.2f}")
    sleep(0.05)
