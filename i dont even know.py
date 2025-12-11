import numpy as np
import math

# ============================================================
#                 MODIFIED DH TRANSFORMATION
# ============================================================
def mDH(a, alpha, d, theta):
    """
    Creates a Modified DH transformation matrix.
    Angles (alpha, theta) are in degrees.
    """
    # Convert degrees -> radians
    alpha = math.radians(alpha)
    theta = math.radians(theta)

    ca = math.cos(alpha)
    sa = math.sin(alpha)
    ct = math.cos(theta)
    st = math.sin(theta)

    T = np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])
    return T

# ============================================================
#                       MDH PARAMETERS
# ============================================================
# mDH = [a, alpha, d, theta0]
mDH_params = np.array([
    [0,     0,   67.5, 180],   # Joint 1
    [-22.5, 90,    0,   0],    # Joint 2
    [113.1, 0,     0,   0],    # Joint 3
    [55,    0,     0,   180]   # Joint 4 (end effector)
])

# Link lengths for IK (extracted from MDH geometry)
l1 = 113.1   # Link from joint 2 to joint 3
l2 = 55.0    # Link from joint 3 to tool

# ============================================================
#                   REACHABILITY FUNCTION
# ============================================================
def reachable(r):
    """Check if distance r is reachable by 2-link mechanism."""
    return abs(l1 - l2) <= r <= (l1 + l2)

# ============================================================
#                FORWARD KINEMATICS (FULL ARM)
# ============================================================
def forward_kinematics(q1, q2, q3):
    """
    Computes FK for joints 1, 2, 3.
    Joint 4 angle is fixed (from MDH table).
    Returns the final 4x4 transform matrix.
    """
    T = np.eye(4)
    qs = [q1, q2, q3, 0]  # joint4 = 0 offset, MDH includes 180° already

    for (a, alpha, d, theta0), q in zip(mDH_params, qs):
        T = T @ mDH(a, alpha, d, theta0 + q)

    return T

# ============================================================
#     INVERSE KINEMATICS FOR JOINT 2 & 3 IN X–Z PLANE
# ============================================================
def twoJointIK(xp, zp):
    """
    Computes IK for a planar 2-link robot using geometry.
    Returns two possible solutions (elbow up/down).
    """
    r = math.hypot(xp, zp)

    if not reachable(r):
        return []

    # Elbow joint: Law of Cosines
    cosQ3 = (r*r - l1*l1 - l2*l2) / (2*l1*l2)
    cosQ3 = max(-1, min(1, cosQ3))   # clamp for numerical safety
    q3 = math.acos(cosQ3)

    # Base-to-target angle
    alpha = math.atan2(zp, xp)

    # Law of Cosines for q2
    beta = math.acos((l1*l1 + r*r - l2*l2) / (2*l1*r))

    # Two configurations
    q2 = alpha - beta
    q2_alt = alpha + beta
    q3_alt = -q3

    return [
        (math.degrees(q2), math.degrees(q3)),
        (math.degrees(q2_alt), math.degrees(q3_alt))
    ]

# ============================================================
#                       TEST EXAMPLE
# ============================================================
if __name__ == "__main__":
    # Example target for IK
    xp = 100
    zp = 50

    print("\n=== Inverse Kinematics ===")
    sols = twoJointIK(xp, zp)
    if sols:
        for i, (q2, q3) in enumerate(sols):
            print(f"Solution {i+1}: q2 = {q2:.2f}°, q3 = {q3:.2f}°")
    else:
        print("Target not reachable.")

    # Example FK with one of the IK solutions
    if sols:
        q1 = 0
        q2, q3 = sols[0]
        T = forward_kinematics(q1, q2, q3)

        print("\n=== Forward Kinematics Result ===")
        print(T)

        print("\nEnd Effector Position:")
        print(f"X = {T[0,3]:.2f}")
        print(f"Y = {T[1,3]:.2f}")
        print(f"Z = {T[2,3]:.2f}")
