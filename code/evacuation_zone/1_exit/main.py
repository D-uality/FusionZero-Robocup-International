import cv2
import numpy as np
import time

from config import *
from camera import *
from perspective_transform import *
from exit_sequence import *
import oled_display
import motors

GPIO.setmode(GPIO.BCM)

laser_sensors.initalise()
touch_sensors.initalise()

try:
    while True:
        start = time.time()

        image = camera.capture_array()
        transformed_image = perspective_transform(image)
        
        exit_sequence()

        FPS = (1) / (time.time() - start)
        print(f"{FPS=:.2f}")


        if X11:
            cv2.imshow("transformed_image", transformed_image)
            cv2.waitKey(1)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    motors.run(0, 0)
    oled_display.reset()

    cv2.destroyAllWindows()
    camera.stop()
    
    GPIO.cleanup()