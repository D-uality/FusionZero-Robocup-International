import time
start_time = time.time()
import cv2
import numpy as np

import gpio
import laser_sensors
import touch_sensors
import oled_display
import camera
import motors

try:
    gpio.initialise()
    oled_display.initialise()
    laser_sensors.initialise()
    touch_sensors.initialise()
    camera.initialise()

    input(f"({time.time() - start_time:.2f}) Press enter to begin program! ")
    oled_display.reset()

    while True:
        start_time = time.time()
        image = camera.capture_array()

        spectral_highlights = cv2.inRange(image, (200, 200, 200), (255, 255, 255))
        kernal = np.ones((7, 7), np.uint8)

        spectral_highlights = cv2.dilate(spectral_highlights, kernal, iterations=1)


        cv2.imshow("image", image)
        cv2.imshow("spectral_highlights", spectral_highlights)

        FPS = (1)/(time.time() - start_time)
        print(f"{FPS=:.2f}")

except KeyboardInterrupt:
    print("Exiting Gracefully")

except Exception as e:
    print(f"Program failed, error: {e}")

finally:
    gpio.cleanup()
    oled_display.reset()
    camera.close()
