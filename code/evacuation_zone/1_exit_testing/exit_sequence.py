import time
import cv2
import numpy as np

from config import *
from camera import *
from cv2_functions import *
import motors
import touch_sensors
import laser_sensors

def exit_sequence():
    while True:
        touch_values = touch_sensors.read([touch_pins[0], touch_pins[1]])
        laser_values = laser_sensors.read([x_shut_pins[0]])

        if touch_values[0] == 0 or touch_values[1] == 0:
            motors.run(-30, -30, 0.4)
            motors.run(30, -30, 0.3)
        elif laser_values[0] > 30:
            motors.run(30, 30, 1.5)
            motors.run(-30, 30, 1.5)
            motors.run(0, 0, 1.5)

            validate_exit()
        else:
            motors.run(29, 30)

        print()

def validate_exit():
    print("Moving backwards, removing black")
    motors.run(-15, -15, 1.3)

    motors.run(15, 15)

    while True:
        laser_values = laser_sensors.read([x_shut_pins[0], x_shut_pins[2]])

        if laser_values[0] < 20 and laser_values[1] < 20:
            motors.run(0, 0)
            break
        else:
            if laser_values[0] < 20: motors.run(5, 15)
            if laser_values[1] < 20: motors.run(15, 5)

        print()

    motors.run(-15, -15, 0.4)
    input()