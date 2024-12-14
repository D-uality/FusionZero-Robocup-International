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
        elif laser_values[0] > 15:
            motors.run(30, 30, 1.5)
            motors.run(-30, 30, 1.5)
            motors.run(0, 0, 1.5)

            validate_exit()
        else:
            motors.run(29, 30)

        print()

def validate_exit():
    print("Moving backwards, removing black")
    motors.run(-10, -10)

    while True:
        image = camera.capture_array()
        image = perspective_transform(image)

        black_mask = cv2.inRange(image, np.array([0, 0, 0]), np.array([50, 50, 50]))
        contours, hierarchy = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
        if not contours: break

        if X11:
            cv2.imshow("image", image)
            cv2.imshow("black_mask", black_mask)
            cv2.waitKey(1)


    motors.run()