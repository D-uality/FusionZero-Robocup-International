import cv2
import numpy as np
import time

from config import *
from camera import *
from perspective_transform import *
from exit_sequence import *
from laser_sensors import *
from touch_sensors import *
import oled_display

GPIO.setmode(GPIO.BCM)

initalise_ToF()
initalise_touch()

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
    run_motors(0, 0)
    oled_display.reset()