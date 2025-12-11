from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import math
import time
import numpy as np


#Servo Setup
factory = PiGPIOFactory()
servo1 = Servo(13, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo2 = Servo(14, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo3 = Servo(15, pin_factory = factory, min_pulse_width = 0.5/1000, max_pulse_width = 2.5/1000)
servo1.value = 0
servo2.value = 0
servo3.value = 0
print("<----- Servo Reset Complete ----->")

#mDH parameters table
mDH = np.array([0, 0, 67.5, 0],
               [-22.5, 90, 0, 180],
               [113.1, 0, 0, 0],
               [55, 0, 0, 0])

L1 = mDH[2][0] #link 2 length 113.1
L2 = mDH[3][0] #link 3 length 55
base_height = mDH[0][2]  #link 1 offset 67.5

#Angle to servo language
def angle_to_servo(angle_deg):
    return max(min(angle_deg / 90.0, 1), -1)

#IK
def IK(x, y, z):
    theta1 = math.degrees(math.atan2(y, x))
    r = math.hypot(x, y)
    z_offset = z - base_height
    
    D = (r ** 2 + z_offset ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    if abs(D) > 1:
        print("Out of reach")
        return None
    
    theta3 = math.degrees(math.atan2(math.sqrt(1 - D ** 2), D))
    theta2 = math.degrees(math.atan2(z_offset, r) - math.atan2(L2 * math.sin(math.radians(theta3)), L1 + L2 * math.cos(math.radians(theta3))))
    return theta1, theta2, theta3

#Move arm to coordinate
def move(x, y, z):
    r = math.hypot(x, y)
    z_offset = z - base_height
    alpha = math.hypot(r, z_offset)
    
    #Reachability
    if alpha > (L1 + L2) or alpha < abs(L1 - L2):
        print("Out of reach")
        return
    
    angles = IK(x, y, z)
    theta1, theta2, theta3 = angles
    servo1.value = angle_to_servo(theta1)
    servo2.value = angle_to_servo(theta2)
    servo3.value = -angle_to_servo(theta3)
    print(f"Moved to angles: {theta1:.2f}, {theta2:.2f}, {theta3:.2f}")

#main
if __name__ == "__main__":
    print("Enter desired shape, or 'q' to quit.")
    while True:
        input = input("shape: ")
        if input == 'q':
            break

        if input == 'triangle':
            x = 100
            y = (0, -50, 0)
            z = (150, 150, 100)
            for i in range(3):
                for k in range(3):
                    move(x, y[k], z[k])
                    time.sleep(0.5)

        elif input == 'star':
            x = 135
            y = (0, 40, -50, 50, -40)
            z = (150, 100, 130, 130, 100)
            for i in range(3):
                for k in range(len(z)):
                    move(x, y[k], z[k])
                    time.sleep(0.5)

        elif input == 'circle':
            x = 135
            radius = 30
            center_y = 0
            center_z = 120
            points = 100

            circle_points = []

            for i in range(points):
                angle = 2 * math.pi * i / points
                y = center_y + radius * math.cos(angle)
                z = center_z + radius * math.sin(angle)
                circle_points.append((x, y, z))

            print(circle_points)
            for k in range(points):
                move(x, circle_points[k][1], circle_points[k][2])
                time.sleep(0.001)

