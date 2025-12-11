from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import math
from time import sleep

# Set up PiGPIO
factory = PiGPIOFactory()

# Servo pins
servo1 = Servo(13, pin_factory=factory)
servo2 = Servo(14, pin_factory=factory)
servo3 = Servo(15, pin_factory=factory)

servo1.value = 0
servo2.value = 0
servo3.value = 0
sleep(3)


servo1.value = -1
sleep(1)
servo2.value = -0.5
sleep(1)
servo3.value = 1
sleep(1)