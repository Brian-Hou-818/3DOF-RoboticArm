from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
import math

factory = PiGPIOFactory()

# ---- MDH Parameters ----
# a, alpha, d, theta
theta1 = 0
theta2 = 0
theta3 = 0
mDH = np.array([[0, 0, 67.5, 180],
                [-22.5, 90, 0, 0],
                [113.1, 0, 0, 0],
                [55, 0, 0, 180]])

# ---- Robot Geometry ----s
baseH = mDH[0][2]
aOffset = mDH[0][0]
l1 = mDH[1][0]
l2 = mDH[2][0]

# ---- Servo Config ----
servoCfg = {
    'joint1': {'neutral': 90, 'dir': 1, 'travelHalf': 90},
    'joint2': {'neutral': 90, 'dir': 1, 'travelHalf': 90},
    'joint3': {'neutral': 90, 'dir': 1, 'travelHalf': 90}}
servo1 = Servo(13, min_pulse_width = 0.5 / 1000, max_pulse_width = 2.5 / 1000, pin_factory = factory)
servo2 = Servo(14, min_pulse_width = 0.5 / 1000, max_pulse_width = 2.5 / 1000, pin_factory = factory)
servo3 = Servo(15, min_pulse_width = 0.5 / 1000, max_pulse_width = 2.5 / 1000, pin_factory = factory)
print("\n<---- Servo Calibration Complete ---->\n")


#converts deg to neutral value
def servoConvert(angle, neutral, direction, travelHalfDeg):
    diff = angle - neutral
    diff = max(-travelHalfDeg, min(travelHalfDeg, diff))
    norm = diff / travelHalfDeg
    return direction * norm

#deg conversion
def deg(rad):
    return rad * 180 / math.pi

def reachable(r):
    return abs(l1 - l2) <= r <= (l1 + l2)

#Joints 2 and 3 IK solving
def twoJointIK(xp, zp):
    r = math.hypot(xp, zp)
    if not reachable(r):
        return []

    cosQ3 = (r ** 2  - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    cosQ3 = max(-1.0, min(1.0, cosQ3))
    q3 = math.acos(cosQ3)

    alpha = math.atan2(zp, xp)
    beta = math.acos((l1 ** 2 + r ** 2 - l2 ** 2) / (2 * l1 * r))
 
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
    floor = -30
    location_dict = {
        'A': [0, -120, floor],
        'B': [31.06, 115.91, floor],
        'C': [60, 103.92, floor],
        'D': [84.85, 84.85, floor],
        'E': [103.92, 60, floor],
        'F': [115.91, 31.06, floor],
        'G': [0, 120, floor],
        'H': [115.91, -31.06, floor],
        'I': [103.92, -60, floor],
        'J': [84.85, -84.95, floor],
        'K': [60, 103.92, floor],
        'L': [31.06, -115.91, floor],

        'M': [0, 100, floor],
        'N': [25.88, 96.59, floor],
        'O': [50, 86.6, floor],
        'P': [70.71, 70.71, floor],
        'Q': [86.6, 50, floor],
        'R': [96.59, 25.88, floor],
        'S': [100, 0, floor],
        'T': [96.59, -25.88, floor],
        'U': [86.6, -50, floor],
        'V': [70.71, -70.71, floor],
        'W': [50, -86.6, floor],
        'X': [25.88, -96.59, floor],

        'Y': [0, 80, floor],
        'Z': [20.71, 77.24, floor],
        '1': [40, 69.28, floor],
        '2': [56.57, 56.57, floor],
        '3': [69.28, 40, floor],
        '4': [77.27, 20.41, floor],
        '6': [80, 0, floor],
        '5': [77.27, -20.41, floor],
        '7': [69.28, -40, floor],
        '8': [56.57, -56.57, floor],
        '9': [40, -69.28, floor],
        '10': [20.71, -77.24, floor]}
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
    print("here")
    print(move_X, move_Y, move_Z)

    theta1 = math.atan2(move_Y, move_X)
    baseDeg = deg(theta1)

    rho = math.hypot(move_X, move_Y)
    xp = rho
    zp = move_Z - baseH

    sols = twoJointIK(xp, zp)
    if not sols:
        print("Target unreachable.\n")
        continue

    t2, t3 = sols[0]
    shoulderDeg = deg(t2)
    elbowDeg = deg(t3)

    servo1.value = servoConvert(baseDeg, servoCfg['joint1']['neutral'], servoCfg['joint1']['dir'], servoCfg['joint1']['travelHalf'])
    servo2.value = servoConvert(shoulderDeg, servoCfg['joint2']['neutral'], servoCfg['joint2']['dir'], servoCfg['joint2']['travelHalf'])
    servo3.value = servoConvert(elbowDeg, servoCfg['joint3']['neutral'], servoCfg['joint3']['dir'], servoCfg['joint3']['travelHalf'])

    print("Base: %.2f, Shoulder: %.2f, Elbow: %.2f\n", baseDeg, shoulderDeg, elbowDeg)
    sleep(0.05)
