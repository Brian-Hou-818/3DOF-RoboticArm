import numpy as np
import math
import time
from sympy import symbols, Eq, solve
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

class ServoController:
    def __init__(self):
        factory = PiGPIOFactory()

        self.servo1 = Servo(13, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        self.servo2 = Servo(14, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        self.servo3 = Servo(15, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        self.servo4 = Servo(16, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        self.servo5 = Servo(17, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

        self.center_all()

    def center_all(self):
        self.servo1.value = 0
        self.servo2.value = 0
        self.servo3.value = 0
        self.servo4.value = 0
        self.servo5.value = 0

    def set_joint_angles(self, q):
        # q[] in radians
        # convert radians → servo value (-1 to 1)
        self.servo1.value = q[0] / (math.pi/2)
        self.servo2.value = q[1] / (math.pi/2)
        self.servo3.value = q[2] / (math.pi/2)
        self.servo4.value = q[3] / (math.pi/2)
        self.servo5.value = q[4] / (math.pi/2)

# =================================================
# 2. Robot Kinematics (DH, FK, Jacobian)
# =================================================

class RobotKinematics:
    def __init__(self):
        #modified DH parameters
        self.mdh = [[180, 0, 0, 67.5],
                    [0, 90, -22.5, 0],
                    [0, 0, 113.1, 0]]

    # 2.1: Compute single MDH transform
    def mdh_transform(self, theta, alpha, r, d):
        T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
                      [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), r*np.sin(theta)],
                      [0,              np.sin(alpha),                np.cos(alpha),               d],
                      [0,              0,                            0,                           1]])
        return T

    # 2.2: Forward kinematics
    def forward_kinematics(self, q):
        T = np.eye(4)
        for i, joint in enumerate(self.mdh):
            theta, alpha, r, d = joint
            T = T @ self.mdh_transform(q[i] + theta, alpha, r, d)
        return T

    # 2.3: Jacobian (geometric)
    def jacobian(self, q):
        """
        Compute Jacobian from DH parameters.
        This is the standard method:
        Jv_i = z_(i-1) × (p_end - p_(i-1))
        Jw_i = z_(i-1)
        """
        # TODO: implement fully
        pass

# =================================================
# 3. Inverse Kinematics (Jacobian IK)
# =================================================

class JacobianIKSolver:
    def __init__(self, kin):
        self.kin = kin

    def step(self, q, target_pos, alpha=0.1):
        """
        q = current joint angles (3x1)
        target_pos = desired XYZ (3x1)
        """
        T = self.kin.forward_kinematics(q)
        current_pos = T[0:3, 3]

        error = target_pos - current_pos

        J = self.kin.jacobian(q)
        dq = alpha * (J.T @ error)

        return q + dq

    def solve(self, initial_q, target_pos, tol=1e-3, max_iters=100):
        q = initial_q.copy()

        for i in range(max_iters):
            T = self.kin.forward_kinematics(q)
            pos = T[0:3, 3]
            if np.linalg.norm(target_pos - pos) < tol:
                break

            q = self.step(q, target_pos)

        return q

# =================================================
# 4. Trajectory Generation (Optional)
# =================================================

def cubic_trajectory(q0, qf, t, tf):
    """
    Returns q(t) via cubic interpolation.
    """
    a0 = q0
    a1 = 0
    a2 = 3*(qf - q0)/(tf**2)
    a3 = -2*(qf - q0)/(tf**3)

    return a0 + a1*t + a2*t**2 + a3*t**3

# =================================================
# 5. Main Control Loop
# =================================================

def main():
    # Initialize modules
    servos = ServoController()
    kin = RobotKinematics()
    ik = JacobianIKSolver(kin)

    # Initial joint angles
    q = np.array([0.0, 0.0, 0.0])   # radians

    # ---- Get user target ----
    x = float(input("Target X: "))
    y = float(input("Target Y: "))
    z = float(input("Target Z: "))
    target_pos = np.array([x, y, z])

    # ---- Solve IK ----
    q_target = ik.solve(q, target_pos)

    # ---- Smooth move with cubic trajectory ----
    tf = 2.0  # seconds
    t = 0
    dt = 0.02

    while t < tf:
        q_t = cubic_trajectory(q, q_target, t, tf)
        servos.set_joint_angles(q_t)
        t += dt
        time.sleep(dt)

    servos.set_joint_angles(q_target)


if __name__ == "__main__":
    main()
