import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform

from config import *

# Source Points
top_left = (10, 100)
bottom_left = (0, HEIGHT-1)
top_right = (WIDTH-10, 100)
bottom_right = (WIDTH, HEIGHT-1)


def perspective_transform(image):
    """Apply perspective transform to the image"""
    # Draw Source Coordinates
    # cv2.circle(image, top_left, 3, (0, 0, 255), -1)
    # cv2.circle(image, bottom_left, 3, (0, 0, 255), -1)
    # cv2.circle(image, top_right, 3, (0, 0, 255), -1)
    # cv2.circle(image, bottom_right, 3, (0, 0, 255), -1)

    # Define points for the perspective transform (source and destination)
    src_points = np.float32([top_left, bottom_left, top_right, bottom_right])
    dst_points = np.float32([[0, 0], [0, HEIGHT], [WIDTH, 0], [WIDTH, HEIGHT]])

    # Calculate the perspective matrix
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Apply the perspective transform to the image
    transformed_image = cv2.warpPerspective(image, matrix, (image.shape[1], image.shape[0]))

    return transformed_image