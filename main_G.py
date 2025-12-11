from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import math
import time

# Set up PiGPIO
factory = PiGPIOFactory()

# Servo pins
servo1 = Servo(13, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo2 = Servo(14, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo3 = Servo(15, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo1.value = 0
servo2.value = 0
servo3.value = 0
print("<----- Servo Reset Complete ----->")

# ---- Arm MDH Parameters ----
L1 = 113.3  # link 2
L2 = 55.0   # link 3
base_height = 67.5  # link 1 offset (Z height)

# Servo calibration
def angle_to_servo(angle_deg):
    """Convert angle in degrees to servo value (-1 to 1)."""
    return max(min(angle_deg / 90.0, 1), -1)

# ---- Inverse Kinematics ----
def ik(x, y, z):
    theta1 = math.degrees(math.atan2(y, x))
    r = math.hypot(x, y)
    z_offset = z - base_height
    
    D = (r**2 + z_offset**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(D) > 1:
        print("Target out of reach")
        return None
    
    theta3 = math.degrees(math.atan2(math.sqrt(1 - D**2), D))
    theta2 = math.degrees(math.atan2(z_offset, r) - math.atan2(L2 * math.sin(math.radians(theta3)), L1 + L2 * math.cos(math.radians(theta3))))
    return theta1, theta2, theta3

# ---- Move Arm ----
def move_arm(x, y, z):
    r = math.hypot(x, y)
    z_offset = z - base_height
    distance = math.hypot(r, z_offset)
    distance = math.hypot(r, z_offset)
    
    # Check reachability
    if distance > (L1 + L2) or distance < abs(L1 - L2):
        print("Target out of reach")
        return
    
    angles = ik(x, y, z)
    theta1, theta2, theta3 = angles
    servo1.value = angle_to_servo(theta1)
    servo2.value = -angle_to_servo(theta2)
    servo3.value = angle_to_servo(theta3)
    print(f"Moved to angles: {theta1:.2f}, {theta2:.2f}, {theta3:.2f}")
    print(angle_to_servo(theta1), angle_to_servo(theta2), angle_to_servo(theta3))

# ---- Interactive Input ----
if __name__ == "__main__":
    print("Enter target coordinates in mm (x y z), or 'q' to quit.")
    while True:
        user_input = input("Coordinates: ")
        if user_input.lower() == 'q':
            break
        # try:
        #     x_str, y_str, z_str = user_input.strip().split()
        #     x, y, z = float(x_str) - 22.5, float(y_str), float(z_str) - 10
        #     move_arm(x, y, z)
        # except ValueError:
        #     print("Invalid input. Please enter three numbers separated by spaces.")

        if user_input == 'triangle':
            x = 100
            y = (0, -50, 0)
            z = (150, 150, 100)
            for i in range(3):
                for k in range(3):
                    move_arm(x, y[k], z[k])
                    time.sleep(0.5)

        elif user_input == 'star':
            x = 135
            y = (0, 40, -50, 50, -40)
            z = (150, 100, 130, 130, 100)
            for i in range(3):
                for k in range(len(z)):
                    move_arm(x, y[k], z[k])
                    time.sleep(0.5)

        elif user_input == 'circle':
            x = 135
            radius = 30  # Radius of the circle
            center_y = 0
            center_z = 120
            num_points = 100  # Number of points to approximate the circle

            circle_points = []

            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                y = center_y + radius * math.cos(angle)
                z = center_z + radius * math.sin(angle)
                circle_points.append((x, y, z))

            print(circle_points)
            for k in range(num_points):
                move_arm(x, circle_points[k][1], circle_points[k][2])
                time.sleep(0.001)

