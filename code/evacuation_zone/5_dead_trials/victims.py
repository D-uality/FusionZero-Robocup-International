import cv2
import numpy as np
from typing import Optional

from config import *

def live(image: np.ndarray) -> Optional[int]:
    spectral_threshold = 200

    kernal_size = 7
    spectral_highlights = cv2.inRange(image, (spectral_threshold, spectral_threshold, spectral_threshold), (255, 255, 255))
    spectral_highlights = cv2.dilate(spectral_highlights, np.ones((kernal_size, kernal_size), np.uint8), iterations=1)

    contours, _ = cv2.findContours(spectral_highlights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours: return None

    largest_contour = max(contours, key=cv2.contourArea)

    x, _, w, _ = cv2.boundingRect(largest_contour)

    if X11: cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)

    return int(x + w/2)

def dead(image: np.ndarray) -> Optional[int]:
    def circularity_check(contour: np.ndarray, threshold: float) -> bool:
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)

        if perimeter == 0: return False

        circularity = (4 * np.pi * area) / (perimeter ** 2)

        if circularity > threshold: return True
        return False
    
    black_threshold = 40

    black_mask = cv2.inRange(image, (0, 0, 0), (black_threshold, black_threshold, black_threshold))
    contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours: return None

    circular_contours = [contour for contour in contours if circularity_check(contour, 0.5)]

    if len(circular_contours) == 0: return None

    largest_contour = max(circular_contours, key=cv2.contourArea)

    x, _, w, _ = cv2.boundingRect(largest_contour)
    if X11: cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)

    return int(x + w/2)