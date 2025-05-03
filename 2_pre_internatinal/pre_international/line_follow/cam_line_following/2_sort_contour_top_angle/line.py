import cv2
import numpy as np

from config import *

y_threshold = (HEIGHT-30, HEIGHT-1)

x_prev = WIDTH // 2
y_prev = HEIGHT

def black_mask(image, display_image):
    """Find the black line"""
    global x_prev, y_prev

    # Mask for black
    black_mask = cv2.inRange(image, (0, 0, 0), (80, 80, 80))

    kernel = np.ones((3,3), np.uint8)
    black_mask = cv2.erode(black_mask, kernel, iterations = 3)

    contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    cv2.drawContours(display_image, contours, -1, (0, 255, 0), 3)

    if len(contours) > 1:
        # Select the lowest contour/contours
        bottom_scores = [cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] for c in contours]
        max_bottom_score = max(bottom_scores)
        bottom_contours = [c for c, score in zip(contours, bottom_scores) if score == max_bottom_score]

        if len(bottom_contours) > 1:
            # If multiple, choose the one closest to the previous position
            def calculate_distance_from_prev(c):
                x, y, w, h = cv2.boundingRect(c)
                contour_x = x + w // 2
                contour_y = y + h

                dx = abs(contour_x - x_prev)  # Absolute x difference
                dy = abs(contour_y - y_prev)  # Absolute y difference
                distance = np.sqrt(dx ** 2 + dy ** 2)
                return distance
            
            contour = min(bottom_contours, key=calculate_distance_from_prev)
        else:
            # Only one contour with the max bottom score
            contour = bottom_contours[0]

    elif contours:
        # Only one contour found
        contour = contours[0]
    else:
        # No contours found
        contour = None

    # Update previous x and y to the center of the selected contour
    if contour is not None:
        x, y, w, h = cv2.boundingRect(contour)
        x_prev = x + w // 2  # Update x_prev to the center of the selected contour
        y_prev = y + h  # Update y_prev to the bottom of the selected contour

    return contour, black_mask

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
        # Find the highest point of the contour (min y value)
        highest_point = min(contour, key=lambda point: point[0][1])
        highest_y = highest_point[0][1]

        # Find all contour points a few pixels below the highest point
        section = [point for point in contour if highest_y <= point[0][1] <= highest_y + 5]

        if len(section) > 0:
            # Calculate the average x of these points
            avg_x = int(np.mean([point[0][0] for point in section]))

            # Calculate the angle to the center of the camera
            dx = avg_x - (WIDTH // 2)  # Difference in x
            dy = HEIGHT - (highest_y + 5)  # Difference in y (camera height to the point)

            # Calculate the angle in radians
            angle_radians = np.arctan2(dy, dx)

            # Convert the angle to degrees
            angle = int(np.degrees(angle_radians))

            # Draw the line from average x to the center of the camera base
            cv2.line(display_image, (avg_x, highest_y + 5), (WIDTH // 2, HEIGHT), (0, 255, 0), 3)
            cv2.putText(display_image, str(angle), (0, HEIGHT-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)

    return angle