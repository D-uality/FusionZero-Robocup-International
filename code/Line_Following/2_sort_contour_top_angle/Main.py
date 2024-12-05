import cv2
import numpy as np
import time
from picamera2 import Picamera2
from libcamera import Transform

from config import *
from transform import *
from line import *

try:
    while True:
        time_start = time.time()  # Start timing FPS 
        image = camera.capture_array()

        # Apply perspective transform
        transformed_image = perspective_transform(image)
        line_image = transformed_image.copy()

        # Find black line
        black_contour, black_image = black_mask(transformed_image, line_image)

        error, angle = 0, 0

        error = calculate_error(black_contour, transformed_image, line_image)
        angle = calculate_angle(black_contour, transformed_image, line_image)

        # Calculate FPS
        fps = int(1 / (time.time() - time_start))

        # Display images
        # cv2.imshow("Original Image", image)
        cv2.imshow("Transformed Image", transformed_image)
        cv2.imshow("Line Image", line_image)
        # cv2.imshow("Black Mask", black_image)
        cv2.waitKey(1)
        
        print(f"FPS: {fps}  |   Error: {error}, Angle: {angle}")
        
except KeyboardInterrupt:
    print("Exiting program!")