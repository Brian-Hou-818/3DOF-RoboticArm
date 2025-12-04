import math
from math import sin, cos, tan, atan, acos, asin, sqrt, pi
import time
import numpy as np

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
print(J)
