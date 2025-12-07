#imports
import math
from sympy import symbols, Eq, solve, sin, cos, tan, atan, acos, asin, sqrt, pi
import time
import numpy as np
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

#setup servos
factory = PiGPIOFactory()
servo1 = Servo(13, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo2 = Servo(14, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo3 = Servo(15, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo4 = Servo(16, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo5 = Servo(17, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo1.value = 0
servo2.value = 0
servo3.value = 0
servo4.value = 0.1
servo5.value = -0.1
print("<----- Servo Reset Complete ----->")

#Functions----

def open():
    servo4.value = -0.3
    servo5.value = 0.3
    return 0

def close():
    servo4.value = 0.1
    servo5.value = -0.1
    return 0

#Code Start----
#modified DH parameters - a, alpha, d, theta
mDH = np.array([[0, 0, 67.5, 180],
                [-22.5, 90, 0, 0],
                [113.1, 0, 0, 0],
                [55, 0, 0, 90]])

while True:
    move_X = float(input("X: "))
    move_Y = float(input("Y: "))
    move_Z = float(input("Z: ")) + 3

    if move_X == 0:
        alpha = 0
    elif move_Y == 0 and move_X < 0:
        alpha = 90
    elif move_Y == 0 and move_X > 0:
        alpha = -90
    else:
        alpha = round(math.atan(move_X / move_Y) * (180 / pi), 2)
    r = round(sqrt(move_Z ** 2 + sqrt(move_X ** 2 + move_Y ** 2) ** 2), 2)
    beta = round(math.acos(sqrt(move_X ** 2 + move_Y ** 2) / r) * (180 / pi), 2)
    print(r, alpha, beta)

    #jacobian
    J = np.array([[round(-r * math.cos(beta) * math.sin(alpha), 4), round(-r * math.sin(beta) * math.cos(alpha), 4), round(math.cos(beta) * math.cos(alpha), 4)],
                [round(r * math.cos(beta) * math.cos(alpha), 4), round(-r * math.sin(beta) * math.sin(alpha), 4), round(math.cos(beta) * math.sin(alpha), 4)],
                [0, round(r * math.cos(beta), 4), round(math.sin(beta), 4)]])
   
    #J_i = np.linalg.inv(J)

    theta1, theta2 = symbols('theta1, theta2')
    eq1 = Eq(mDH[2][0] * cos(theta1) + mDH[3][0] * cos(theta1 + theta2), move_X - mDH[1][0])
    eq2 = Eq(mDH[2][0] * sin(theta1) + mDH[3][0] * sin(theta1 + theta2), move_Y - mDH[0][2])
    d = solve((eq1, eq2), (theta1, theta2))
    list1 = list(d[0])
    print(alpha, list1[0], list1[1])

    servo1.value = alpha / 90
    if list1[0] > list1[1]:
        servo2.value = -list1[0] / 90
        servo3.value = list1[1] / 90
        print(alpha, list1[0], list1[1])

    else:
        servo2.value = -list1[1] / 90
        servo3.value = list1[0] / 90
        print(alpha, list1[1], list1[0])
