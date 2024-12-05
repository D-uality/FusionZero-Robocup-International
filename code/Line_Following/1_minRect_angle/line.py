import cv2
import numpy as np

from config import *

y_threshold = (HEIGHT-30, HEIGHT-1)

def black_mask(image, display_image):
    """Find the black line"""
    # Mask for black
    black_mask = cv2.inRange(image, (0, 0, 0), (80, 80, 80))

    kernel = np.ones((3,3), np.uint8)
    black_mask = cv2.erode(black_mask, kernel, iterations = 3)

    contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    cv2.drawContours(display_image, contours, -1, (0, 255, 0), 3)

    return contours, black_mask

def calculate_error(contour, image, display_image):
    error = 0
    
    if contour is not None:
        # Section the contour based on y-threshold
        section = [point for point in contour if y_threshold[0] <= point[0][1] <= y_threshold[1]]
        
        # Check if we have any points in the section
        if len(section) > 0:
            # Create a bounding box for the section
            x, y, w, h = cv2.boundingRect(np.array(section))  # Use np.array to convert section to the right format
            x_mid = int(x + (w / 2))
            error = int(x_mid - (WIDTH / 2)) 

            # Draw the midline
            cv2.line(display_image, (x_mid, y_threshold[0]), (x_mid, y_threshold[1]), (255, 0, 0), 3)
            cv2.putText(display_image, str(error), (0, HEIGHT), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 1)	
    
    return error

def calculate_angle(contour, image, display_image):
    angle = 0

    if contour is not None:
        [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)

        # Calculate the angle (in degrees) relative to the horizontal axis
        angle = int(np.arctan2(vy, vx) * (180 / np.pi))

        if angle > 0:
            angle = 90 - angle
        else:
            angle = -1 * (90 + angle)

        # Draw the fitted line on the image for visualization
        rows, cols = image.shape[:2]
        x1, y1 = int(x0 - vx * 200), int(y0 - vy * 200)
        x2, y2 = int(x0 + vx * 200), int(y0 + vy * 200)

        cv2.line(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(display_image, str(angle), (0, HEIGHT-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)	
    
    return angle
    