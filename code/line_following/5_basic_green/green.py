import cv2
import numpy as np

from config import *

min_green_area = 100

green_threshold = ((40, 50, 50), (90, 255, 255))

def green_mask(image, display_image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv_image, green_threshold[0], green_threshold[1])

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    green_mask = cv2.erode(green_mask, kernel, iterations=2)

    contours, _ = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_green_area]

    if contours is not None and display_image is not None:
        cv2.drawContours(display_image, contours, -1, (0, 255, 0), 3)

    return contours, green_mask

def validate_green_contour(contour, black_image):
    # Get the minAreaRect and its 4 corners
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)  # Get the 4 corners of the rectangle
    box = np.int0(box)

    # Sort corners by y-coordinate to identify the top two corners
    sorted_box = sorted(box, key=lambda x: x[1])  # Sort by y-coordinate (top to bottom)
    top_left = sorted_box[0]  # Top-left corner (lower y-coordinate)
    top_right = sorted_box[1]  # Top-right corner

    # Calculate the midpoint of the top two corners
    midpoint_x = (top_left[0] + top_right[0]) // 2
    midpoint_y = (top_left[1] + top_right[1]) // 2
    midpoint = (midpoint_x, midpoint_y)

    # Check if there is black above the midpoint (10 pixels above)
    if 0 <= midpoint[0] - 30 < black_image.shape[1] and 0 <= midpoint[1] - 30 < black_image.shape[0]:
        black_above_midpoint = black_image[midpoint[1] - 30, midpoint[0]] == 255
    else:
        black_above_midpoint = False

    # If black is above the midpoint, return the 4 corners
    if black_above_midpoint:
        return box  # Return the 4 corners of the valid minAreaRect

    return None  # If no black above midpoint, return None

def green_sign(valid_rects, black_image, display_image):
    if len(valid_rects) == 1:
        # Only one valid contour: check the lowest corners for the turn
        rect = valid_rects[0]
        
        # Find the lowest corners based on the y-coordinate (bottom-left and bottom-right corners)
        sorted_rect = sorted(rect, key=lambda x: x[1], reverse=True)  # Sort by y-coordinate (top to bottom)
        bottom_corners = sorted_rect[:2]  # Get the two lowest corners
        
        # Determine which one is the left and which is the right based on x-coordinate
        if bottom_corners[0][0] < bottom_corners[1][0]:
            left_bottom = bottom_corners[0]
            right_bottom = bottom_corners[1]
        else:
            left_bottom = bottom_corners[1]
            right_bottom = bottom_corners[0]
        
        # Check if there's black a few pixels to the left of the bottom-left corner (right turn)
        left_check_x = left_bottom[0] - 10
        left_check_y = left_bottom[1]
        
        if 0 <= left_check_y < HEIGHT and 0 <= left_check_x < WIDTH:
            if black_image[left_check_y, left_check_x] == 255:
                cv2.putText(display_image, "Right Turn", (left_bottom[0], left_bottom[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                return "Right"
        else:
            # If no black on the left, check the bottom-right corner (left turn)
            right_check_x = right_bottom[0] + 10
            right_check_y = right_bottom[1]
            if 0 <= right_check_y < HEIGHT and 0 <= right_check_x < WIDTH:
                if black_image[right_check_y, right_check_x] == 255:
                    cv2.putText(display_image, "Left Turn", (right_bottom[0], right_bottom[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    return "Left"
                else:
                    # Ignore if no black detected at either side
                    return "None"
                
    elif len(valid_rects) == 2:
        # Two contours detected: check the centers for a U-turn
        rect1 = valid_rects[0]
        rect2 = valid_rects[1]
        
        # Get the center of each contour
        rect1_center = (int(np.mean([point[0] for point in rect1])), int(np.mean([point[1] for point in rect1])))
        rect2_center = (int(np.mean([point[0] for point in rect2])), int(np.mean([point[1] for point in rect2])))
        
        # Calculate the midpoint between the two centers
        mid_x = (rect1_center[0] + rect2_center[0]) // 2
        mid_y = (rect1_center[1] + rect2_center[1]) // 2
        mid_point = (mid_x, mid_y)
        
        # Check for black at the midpoint of the two centers (for U-turn)
        if black_image[mid_y, mid_x] == 255:
            cv2.putText(display_image, "U-Turn", (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            return "U-Turn"
        else:
            # Ignore if no black detected at the midpoint
            return "None"