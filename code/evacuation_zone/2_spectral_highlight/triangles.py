import time
import cv2
import numpy as np

from config import *
import laser_sensors
import touch_sensors
import oled_display
import camera
import motors

def align(base_speed, tolerance, text, time_step=None):
    direction = 1
    while True:
        print(f"({text})    |    ", end="")

        laser_values = laser_sensors.read()

        if laser_values[1] < 25: motors.run(-base_speed, -base_speed)
        else:
            if time_step is None: motors.run(base_speed * direction, -base_speed * direction)
            else:
                motors.run(base_speed * direction, -base_speed * direction, time_step)
                motors.run(0, 0, time_step)

        image = camera.capture_array()

        green = cv2.inRange(image, (20, 80, 10), (80, 160, 50))
        green = cv2.dilate(green, np.ones((7, 7), np.uint8), iterations=1)
        contours, _ = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            if X11: cv2.imshow("image", image)
            print()
            continue

        largest_contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest_contour) < 2000:
            if X11: cv2.imshow("image", image)
            print()
            continue

        x, y, w, h = cv2.boundingRect(largest_contour)

        direction = -1 if WIDTH/2 - int(x+ w/2) >= 0 else 1

        if X11:
            cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
            cv2.circle(image, (int(x + w / 2), int(y + h / 2)), 3, (255, 0, 0), 1)
            cv2.imshow("image", image)

        if WIDTH/2 - (x + w / 2) < tolerance and WIDTH/2 - (x + w / 2) > -tolerance: break

        print(f"{WIDTH/2 - (x + w / 2)}")


def find(base_speed):
    motors.claw_step(180, 0)

    print("(TRIANGLE SEARCH) Initial alignment")
    align(base_speed=base_speed * 0.4, tolerance=10, text="Initial Alignment")

    motors.run_until(base_speed, base_speed, laser_sensors.read, 1, "<=", 35)
    motors.run_until(-base_speed, -base_speed, laser_sensors.read, 1, ">=", 35)

    print("(TRIANGLE SEARCH) Fine alignment")
    align(base_speed=base_speed * 0.2, tolerance=3, text="Fine Alignment", time_step=0.1)

    motors.run(0, 0)