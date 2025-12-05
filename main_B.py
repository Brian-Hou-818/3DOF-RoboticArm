#imports
import math
from math import sin, cos, tan, atan, acos, asin, sqrt, pi
from sympy import symbols, Eq, solve
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

#current position
O = np.array([[servo1.value],
              [servo2.value],
              [servo3.value]])

dO = np.array([[0],
               [0],
               [0],
               [0]])

h = 0.00001

#target position
T = O + dO * h

#Functions----

def getJacobianT():
    J_A = np.cross(rotAxisA, O - servo1.value)
    J_B = np.cross(rotAxisB, O - servo2.value)
    J_C = np.cross(rotAxisC, O - servo3.value)
    J = np.array([[],[],[]])
    J.addColumn(J_A)
    J.addColumn(J_B)
    J.addColumn(J_C)
    return J.transpose()

def getDelta():
    Jt = getJacobianT()
    V = T - O
    dO = Jt * V
    return dO

def JacobianIK(mDH):
    while abs(mDH - T) > 0.1:
        dO = getDelta()
        mDH += dO * h

#claw functions
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

move_X = int(input("X: "))
move_Y = int(input("Y: "))
move_Z = int(input("Z: "))

r = round(sqrt(move_Z ** 2 + sqrt(move_X ** 2 + move_Y ** 2) ** 2), 2)
alpha = round(math.atan(move_X / move_Y) * (180 / pi), 2)
beta = round(math.acos(sqrt(move_X ** 2 + move_Y ** 2) / r) * (180 / pi), 2)
print(r, alpha, beta)

#jacobian
J = np.array([[round(-r * math.cos(beta) * math.sin(alpha), 4), round(-r * math.sin(beta) * math.cos(alpha), 4), round(math.cos(beta) * math.cos(alpha), 4)],
              [round(r * math.cos(beta) * math.cos(alpha), 4), round(-r * math.sin(beta) * math.sin(alpha), 4), round(math.cos(beta) * math.sin(alpha), 4)],
              [0, round(r * math.cos(beta), 4), round(math.sin(beta), 4)]])

#J_i = np.linalg.inv(J)

theta1, theta2 = symbols('theta1, theta2')
eq1 = Eq(atan(beta - mDH[3][0] * sin((theta1 + theta2) * pi / 180)) / (alpha - mDH[2][0] * cos((theta1 + theta2) * pi / 180)), theta1)
eq2 = Eq(theta1 + theta2, sqrt(alpha ** 2 + beta ** 2))
d = solve((eq1, eq2), (theta1, theta2))
list1 = list(d[0])

servo1.value = alpha / 90
if list1[0] > list1[1]:
    servo2.value = list1[0]
    servo3.value = list1[1]

else:
    servo2.values = list1[1]
    servo3.values = list1[0]