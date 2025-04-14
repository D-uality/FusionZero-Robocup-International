from config import *
import numpy as np
import cv2

import camera
import motors
import laser_sensors

def find(image):
    def circularity_check(contour, threshold):
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)

        if perimeter == 0: return False
        
        circularity = (4 * np.pi * area) / (perimeter ** 2)
        
        if circularity > threshold: return True
        return False

    black_image = cv2.inRange(image, (0, 0, 0), (40, 40, 40))
    contours, _ = cv2.findContours(black_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours is None: return None

    circular_contours = [contour for contour in contours if circularity_check(contour, 0.5)]

    if len(circular_contours) == 0: return None

    largest_contour = max(circular_contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)

    if X11: cv2.drawContours(image, [largest_contour], -1, (0, 0, 255), 1)

    return int(x + w/2)