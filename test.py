import math
import time
import numpy as np
from sympy import symbols, Eq, solve, sin, cos, tan, atan, acos, asin, sqrt, pi

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
eq1 = Eq(atan(beta - 55 * sin((theta1 + theta2) * pi / 180)) / (alpha - 113.1 * cos((theta1 + theta2) * pi / 180)), theta1)
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
