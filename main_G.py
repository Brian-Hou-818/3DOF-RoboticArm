from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
import math

factory = PiGPIOFactory()

import numpy as np

# ---- MDH Parameters ----
# a, alpha, d, theta
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

# ---- Servo Configuration ----
servoCfg = {
    'joint1': {'neutral': 90, 'dir': 1, 'travelHalf': 90},
    'joint2': {'neutral': 90, 'dir': 1, 'travelHalf': 90},
    'joint3': {'neutral': 90, 'dir': 1, 'travelHalf': 90},
    'clawLeft': {'neutral': 90, 'dir': 1, 'travelHalf': 60},
    'clawRight': {'neutral': 90, 'dir': -1, 'travelHalf': 60}
}

servo1 = Servo(13, min_pulse_width = 0.5 / 1000, max_pulse_width = 2.5 / 1000, pin_factory = factory)
servo2 = Servo(14, min_pulse_width = 0.5 / 1000, max_pulse_width = 2.5 / 1000, pin_factory = factory)
servo3 = Servo(15, min_pulse_width = 0.5 / 1000, max_pulse_width = 2.5 / 1000, pin_factory = factory)

print("<---- Servo Calibration Complete ---->")

#converts deg to neutral value
def servoConvert(angleDeg, neutral, direction, travelHalfDeg):
    diff = angleDeg - neutral
    diff = max(-travelHalfDeg, min(travelHalfDeg, diff))
    norm = diff / travelHalfDeg
    return direction * norm

#deg conversion
def deg(rad): return rad * 180 / math.pi
def rad(deg): return deg * math.pi / 180

def reachable(r):
    return l2 - l1 <= r <= l1 + l2

#Joints 2 and 3 IK solving
def twoJointIK(xp, zp):
    r = math.hypot(xp, zp)
    if not reachable(r):
        return []

    cosQ3 = (r * r - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    cosQ3 = max(-1.0, min(1.0, cosQ3))
    q3 = math.acos(cosQ3)

    alpha = math.atan2(zp, xp)
    beta = math.acos((l1 ** 2 + r * r - l2 ** 2) / (2 * l1 * r))

    q2 = alpha - beta
    q2_alt = alpha + beta
    q3_alt = -q3

    return [(q2, q3), (q2_alt, q3_alt)]

def morseToLetter(morse):
    morse_dict = {
    '._': 'A',
    '_...': 'B',
    '_._.': 'C',
    '_..': 'D',
    '.': 'E',
    '.._.': 'F',
    '__.': 'G',
    '....': 'H',
    '..': 'I',
    '.___': 'J',
    '_._': 'K',
    '._..': 'L',
    '__': 'M',
    '_.': 'N',
    '___': 'O',
    '.__.': 'P',
    '__._': 'Q',
    '._.': 'R',
    '...': 'S',
    '_': 'T',
    '.._': 'U',
    '..._': 'V',
    '.__': 'W',
    '_.._': 'X',
    '_.__': 'Y',
    '__..': 'Z',
    '_____': '0',
    '.____': '1',
    '..___': '2',
    '...__': '3',
    '...._': '4',
    '.....': '5',
    '_....': '6',
    '__...': '7',
    '___..': '8',
    '____.': '9',}

    return str(morse_dict[morse])

def location(letter):
    location_dict = {
        'A': [, , -15],
        'B': [, , -15],
        'C': [, , -15],
        'D': [, , -15],
        'E': [, , -15],
        'F': [, , -15],
        'G': [, , -15],
        'H': [, , -15],
        'I': [, , -15],
        'J': [, , -15],
        'K': [, , -15],
        'L': [, , -15],

        'M': [, , -15],
        'N': [, , -15],
        'O': [, , -15],
        'P': [, , -15],
        'Q': [, , -15],
        'R': [, , -15],
        'S': [, , -15],
        'T': [, , -15],
        'U': [, , -15],
        'V': [, , -15],
        'W': [, , -15],
        'X': [, , -15],

        'Y': [, , -15],
        'Z': [, , -15],
        '1': [, , -15],
        '2': [, , -15],
        '3': [, , -15],
        '4': [, , -15],
        '5': [, , -15],
        '6': [, , -15],
        '7': [, , -15],
        '8': [, , -15],
        '9': [, , -15],
        '10': [, , -15]}
    return location_dict[letter]

# Main Loop
while True:
    inputLetter = str(input("Morse Code: "))
    # move_X = float(input("X: "))
    # move_Y = float(input("Y: "))
    # move_Z = float(input("Z: ")) + 30

    move_X = float(location(morseToLetter(inputLetter))[0])
    move_Y = float(location(morseToLetter(inputLetter))[1])
    move_Z = float(location(morseToLetter(inputLetter))[2])

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
    shoulderDeg = deg(t2)
    elbowDeg = deg(t3)

    servo1.value = servoConvert(baseDeg, servoCfg['joint1']['neutral'], servoCfg['joint1']['dir'], servoCfg['joint1']['travelHalf'])
    sleep(0.3)
    servo2.value = servoConvert(shoulderDeg, servoCfg['joint2']['neutral'], servoCfg['joint2']['dir'], servoCfg['joint2']['travelHalf'])
    sleep(0.3)
    servo3.value = servoConvert(elbowDeg, servoCfg['joint3']['neutral'], servoCfg['joint3']['dir'], servoCfg['joint3']['travelHalf'])
    sleep(0.3)

    print(f"Base: %.2f, Shoulder: %.2f, Elbow: %.2f\n", baseDeg, shoulderDeg, elbowDeg)
    sleep(0.05)
