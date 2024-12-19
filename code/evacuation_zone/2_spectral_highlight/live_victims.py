from config import *
import numpy as np
import cv2

import camera
import motors

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
    image = camera.capture_array()
    x, _ = find(image, 7)

    error = WIDTH - x
    turn = int(error * kP)
    motors.run(v + turn, v - turn)