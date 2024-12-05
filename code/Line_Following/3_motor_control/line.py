import cv2
import numpy as np

from config import *

y_threshold = (HEIGHT // 2 - 20, HEIGHT // 2 + 20)
min_area = 1000

def black_mask(image, display_image):
    """Find the black line"""
    global x_prev, y_prev

    # Mask for black
    black_mask = cv2.inRange(image, (0, 0, 0), (80, 80, 80))

    contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

    if len(contours) == 1:
        contour = contours[0]
    elif len(contours) == 0:
        contour = None
    elif len(contours) > 1:
        # Select the lowest contour/contours
        bottom_scores = [cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] for c in contours]
        max_bottom_score = max(bottom_scores)
        bottom_contours = [c for c, score in zip(contours, bottom_scores) if score == max_bottom_score]

        contour = bottom_contours[0]

    if contour is not None and display_image is not None:
        cv2.drawContours(display_image, contours, -1, (0, 255, 0), 3)

    return contour, black_mask

def calculate_error(image, display_image):
    error = 0

    cropped_image = image[y_threshold[0]:y_threshold[1], :]
    
    contour, _ = black_mask(cropped_image, None)

    if contour is not None:
        x, y, w, h = cv2.boundingRect(contour)

        # Adjust bounding box coordinates to the original image coordinates
        y += y_threshold[0]
        cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 0), 3)

        # Calculate error as the difference between the bounding box center and the image center
        x_mid = int(x + (w / 2))
        error = int(x_mid - (image.shape[1] / 2))

        # Draw the midline for visualization
        cv2.line(display_image, (x_mid, y_threshold[0]), (x_mid, y_threshold[1]), (255, 0, 0), 3)
        cv2.putText(display_image, str(error), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    return error

def calculate_angle(contour, display_image):
    angle = 90

    if contour is not None:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)  # Adjust epsilon for polygon approximation
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Sort the points based on the y-coordinate (topmost first)
        sorted_points = sorted(approx, key=lambda point: point[0][1])

        # Get the top two corners (smallest y values)
        top_two_corners = sorted_points[:2]

        if len(top_two_corners) == 2:
            # Find the midpoint of the x-coordinates of the top two corners
            x_mid = int((top_two_corners[0][0][0] + top_two_corners[1][0][0]) / 2)

            # Find the highest y value of the top two corners
            highest_y = min(top_two_corners[0][0][1], top_two_corners[1][0][1])

            # Calculate the angle using the midpoint and the center of the image
            dx = x_mid - (WIDTH // 2)  # Difference in x
            dy = HEIGHT - highest_y    # Difference in y (camera height to the point)

            # Calculate the angle in radians
            angle_radians = np.arctan2(dy, dx)

            # Convert the angle to degrees
            angle = int(np.degrees(angle_radians))

            # Draw the line from the midpoint to the center of the camera base
            cv2.line(display_image, (x_mid, highest_y), (WIDTH // 2, HEIGHT), (0, 255, 0), 3)
            cv2.putText(display_image, str(angle), (0, HEIGHT-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)

    return angle
