from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import math

# ---- Servo setup ----
factory = PiGPIOFactory()
servo_kwargs = {'pin_factory': factory}

joint1 = Servo(13, **servo_kwargs)
joint2 = Servo(14, **servo_kwargs)
joint3 = Servo(15, **servo_kwargs)

# ---- Servo direction inversion flags ----
invert_joint1 = False
invert_joint2 = False
invert_joint3 = False

# MDH = np.array([
#     [-22.5,    90,  67.5, 0.0],    # Joint 1
#     [113.3,   0.0,   0.0, 0.0],    # Joint 2
#     [55.0,    0.0,   0.0, 0.0]     # Joint 3
# ])

# ---- Link lengths (mm) ----
d1 = 67.5   # base height
a2 = 113.3  # shoulder to elbow
a3 = 55     # elbow to pointer

# ---- Helper function ----
def deg_to_servo_angle(theta_deg, invert=False):
    """Convert degrees to servo value (-1 to 1) and optionally invert"""
    value = max(min(theta_deg / 90.0 - 1, 1), -1)
    if invert:
        value = -value
    return value

# ---- Inverse Kinematics ----
def inverse_kinematics(x, y, z):
    """Compute θ1, θ2, θ3 in degrees for a reachable point"""
    # Base rotation
    theta1 = math.atan2(y, x)
    
    # Planar distance for joints 2 and 3
    r = math.sqrt(x**2 + y**2)
    z_eff = z - d1
    R = math.sqrt(r**2 + z_eff**2)
    
    # Check reachability
    if R > (a2 + a3) or R < abs(a2 - a3):
        raise ValueError("Target is out of reach")
    
    # Elbow (theta3) using cosine law
    cos_theta3 = (R**2 - a2**2 - a3**2) / (2 * a2 * a3)
    theta3 = math.atan2(math.sqrt(1 - cos_theta3**2), cos_theta3)  # elbow-up
    
    # Shoulder (theta2)
    theta2 = math.atan2(z_eff, r) - math.atan2(a3 * math.sin(theta3), a2 + a3 * math.cos(theta3))
    
    # Convert to degrees
    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)

# ---- Move function ----
def move_to(x, y, z):
    try:
        t1, t2, t3 = inverse_kinematics(x, y, z)
        print(f"Moving to angles: θ1={t1:.2f}, θ2={t2:.2f}, θ3={t3:.2f}")
        
        joint1.value = deg_to_servo_angle(t1, invert_joint1)
        joint2.value = deg_to_servo_angle(t2, invert_joint2)
        joint3.value = deg_to_servo_angle(t3, invert_joint3)
        
        sleep(1)
    except ValueError as e:
        print(e)

# ---- Main loop ----
if __name__ == "__main__":
    while True:
        try:
            x = float(input("Enter X (mm): "))
            y = float(input("Enter Y (mm): "))
            z = float(input("Enter Z (mm): "))
            move_to(x, y, z)
        except KeyboardInterrupt:
            print("Exiting...")
            break
