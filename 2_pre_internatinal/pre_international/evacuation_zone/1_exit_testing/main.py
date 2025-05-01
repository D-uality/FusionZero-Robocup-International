import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

from config import *
from camera import *
from cv2_functions import *
from exit_sequence import *
import oled_display
import motors

GPIO.setmode(GPIO.BCM)

laser_sensors.initalise()
touch_sensors.initalise()

try:
    while True:
        exit_sequence()


except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    motors.run(0, 0)
    oled_display.reset()

    cv2.destroyAllWindows()
    camera.stop()

    GPIO.cleanup()