import numpy as np
import math


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

def reachable(r):
    return l2 - l1 <= r <= l1 + l2

def deg(rad): return rad * 180 / math.pi
def rad(deg): return deg * math.pi / 180

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

while True:
    move_X = float(input("X: "))
    move_Y = float(input("Y: "))
    move_Z = float(input("Z: ")) + 30

    theta1 = math.atan2(move_Y, move_X)
    baseDeg = deg(theta1)

    rho = math.hypot(move_X, move_Y)
    xp = rho - aOffset
    zp = move_Z - baseH

    sols = twoJointIK(xp, zp)
    if sols:
        print("Target reachable")
    else:
        print("Target unreachable.\n")

    t2, t3 = sols[0]
    shoulderDeg = deg(t2)
    elbowDeg = deg(t3)

