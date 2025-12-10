from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
import math

factory = PiGPIOFactory()

servoCfg = {
    'joint1': {'neutral': 90, 'dir': 1, 'travelHalf': 90},
    'joint2': {'neutral': 90, 'dir': 1, 'travelHalf': 90},
    'joint3': {'neutral': 90, 'dir': 1, 'travelHalf': 90}}
servo1 = Servo(13, min_pulse_width = 0.5 / 1000, max_pulse_width = 2.5 / 1000, pin_factory = factory)
servo2 = Servo(14, min_pulse_width = 0.5 / 1000, max_pulse_width = 2.5 / 1000, pin_factory = factory)
servo3 = Servo(15, min_pulse_width = 0.5 / 1000, max_pulse_width = 2.5 / 1000, pin_factory = factory)

sleep(3)
servo2.value = 0.5
sleep(5)