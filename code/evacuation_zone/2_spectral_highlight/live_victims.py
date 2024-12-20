from config import *
import numpy as np
import cv2

import camera
import motors
import laser_sensors

def find(image, kernal_size):
    spectral_highlights = cv2.inRange(image, (200, 200, 200), (255, 255, 255))
    spectral_highlights = cv2.dilate(spectral_highlights, np.ones((kernal_size, kernal_size), np.uint8), iterations=1)

    contours, _ = cv2.findContours(spectral_highlights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours: return None, None

    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)

    if X11: cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)

    return int(x + w/2), int(y + h/2)

def route(v, kP):
    distance = laser_sensors.read([x_shut_pins[1]])
    while distance is None: distance = laser_sensors.read([x_shut_pins[1]])

    while distance[0] > 15:
        image = camera.capture_array()
        distance  = laser_sensors.read([x_shut_pins[1]])
        x, _ = find(image, 7)

        if x is None: return False

        error = WIDTH / 2 - x
        turn = int(error * kP)
        motors.run(v - turn, v + turn)

        cv2.imshow("image", image)
        print(f"{error=} {turn=}")

    motors.run(0, 0)

    return True
