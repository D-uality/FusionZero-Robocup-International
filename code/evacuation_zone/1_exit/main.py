import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import VL53L1X

from config import *
from camera import *
from perspective_transform import *
from exit_sequence import *

GPIO.setmode(GPIO.BCM)

for pin in front_touch_pins:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

for pin in back_touch_pins:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        start = time.time()

        image = camera.capture_array()
        transformed_image = perspective_transform(image)

        FPS = (1) / (time.time() - start)
        print(f"{FPS=:.2f}")

        exit_sequence()

        if X11:
            cv2.imshow("transformed_image", transformed_image)
            cv2.waitKey(1)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    cv2.destroyAllWindows()
    camera.stop()
    GPIO.cleanup()
