#imports
import math
from math import sin, cos, tan, atan, acos, asin, sqrt, pi
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
#modified DH parameters
#a, alpha, d, theta
mDH = np.array([[0, 0, 67.5, 180],
                [-22.5, 90, 0, 0],
                [113.1, 0, 0, 0],
                [55, 0, 0, 90]])

move_X = int(input("X: "))
move_Y = int(input("Y: "))
move_Z = int(input("Z: "))

r = round(sqrt(move_Z ** 2 + sqrt(move_X ** 2 + move_Y ** 2) ** 2), 2)
alpha = round(atan(move_X / move_Y) * (180 / pi), 2)
beta = round(acos(sqrt(move_X ** 2 + move_Y ** 2) / r) * (180 / pi), 2)
print(r, alpha, beta)

#jacobian
J = np.array([[round(-r * cos(beta) * sin(alpha), 4), round(-r * sin(beta) * cos(alpha), 4), round(cos(beta) * cos(alpha), 4)],
              [round(r * cos(beta) * cos(alpha), 4), round(-r * sin(beta) * sin(alpha), 4), round(cos(beta) * sin(alpha), 4)],
              [0, round(r * cos(beta), 4), round(sin(beta), 4)]])

J_i = np.linalg.inv(J)