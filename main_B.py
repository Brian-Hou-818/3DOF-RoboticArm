from sympy import symbols, Eq, solve
import math
import time
import numpy as np
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()
servo1 = Servo(13, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo2 = Servo(14, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo3 = Servo(15, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo4 = Servo(16, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo5 = Servo(17, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo1.value = 0
servo2.value = 0
servo3.value = 0
servo4.value = 0
servo5.value = 0

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
def get_values(Fdeg, Ideg, Ft, It, Fvel, Ivel):
    a0, a1, a2, a3 = symbols('a0, a1, a2, a3')
    eq1 = Eq((a0 + a1 * Ft + a2 * Ft ** 2 + a3 * Ft ** 3), Fdeg)
    eq2 = Eq((a0 + a1 * It + a2 * It ** 2 + a3 * It ** 3), Ideg)
    eq3 = Eq((a1 + 2 * a2 * It + 3 * a3 * It ** 2), Ivel)
    eq4 = Eq((a1 + 2 * a2 * Ft + 3 * a3 * Ft ** 2), Fvel)
    d = solve((eq1, eq2, eq3, eq4), (a0, a1, a2, a3))
    list = [*d.values()]
    return list

def getJacobianT():
    J_A = np.cross(rotAxisA, O - jointAPos)
    J_B = CrossProduct(rotAxisB, endEffectorPos - jointBPos)
    J_C = np.cross(rotAxisC, O - jointCPos)
    J = np.array([[1],[2],[]])
    J.addColumn(J_A)
    J.addColumn(J_B)
    J.addColumn(J_C)
    return J.transpose()

def getDelta():
    Jt = getJacobianT()
    V = T - O
    dO = Jt * V
    return dO

def JacobianIK(O):
    while abs(O - T) > 0.1:
        dO = getDelta()
        O += dO * h

def open():
    servo4.value = -0.3
    servo5.value = 0.3
    return 0

def close():
    servo4.value = -0.1
    servo5.value = -0.1
    return 0

#Code Start----
#modified DH parameters
mdh_param = [[],
             [],
             [],
             []]

move_X = int(input("X: "))
move_Y = int(input("Y: "))
move_Z = int(input("Z: "))

if move_X == 0:
    move_X = 0.001
if move_Y == 0:
    move_Y = 0.001
if move_Z == 0:
    move_Z = 0.001

Fdeg = round(math.degrees(math.atan(move_Z / math.sqrt(move_X ** 2 + move_Y ** 2))), 2)#calculate Z angle
Ideg = 0
Fvel = 0
Ivel = 0
Ft = int(input("Time: ")) #Time
It = 0 #initial time
values = get_values(Fdeg, Ideg, Ft, It, Fvel, Ivel)
print(values)

angle = round(values[0] + values[1] * Ft + values[2] * Ft ** 2 + values[3] * Ft ** 3, 2)
print("angle =", angle)

Ideg = Fdeg


servo1.value = servo1.value + (values[0] / 90)
servo2.value = servo2.value + (values[1] / 90)
servo3.value = servo3.value + (values[2] / 90)
servo4.value = servo4.value + (values[3] / 90)
time.sleep(Ft + 0.5)
