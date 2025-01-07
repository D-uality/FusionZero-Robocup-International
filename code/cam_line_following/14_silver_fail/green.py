import cv2
import numpy as np

from config import *

min_green_area = int(HEIGHT * WIDTH/768)
black_height = int(HEIGHT * 0.12)
green_threshold = ((40, 50, 50), (90, 255, 255))
green_turn = None

def green_mask(image, display_image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv_image, green_threshold[0], green_threshold[1])

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    green_mask = cv2.erode(green_mask, kernel, iterations=2)

    contours, _ = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_green_area]

    if contours is not None and display_image is not None:
        cv2.drawContours(display_image, contours, -1, (0, 255, 0), 2)

    return contours, green_mask

def validate_green_contour(contour, black_image, display_image, check_distance=10, region_size=3):
    """
    Validates a green contour by checking for black pixels in a small region
    above the bounding rect's top edge, perpendicular to the edge.

    Args:
        contour: Contour to validate.
        black_image: Binary image where black regions are marked with value 255.
        check_distance: Distance in pixels to check above the top edge.
        region_size: Size of the region (square) to check around the target point.

    Returns:
        box: The 4 corners of the valid minAreaRect, or None if not valid.
    """
    # Get the minAreaRect and its 4 corners
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)  # Get the 4 corners of the rectangle
    box = np.int0(box)

    # Sort corners by y-coordinate to identify the top two corners
    sorted_box = sorted(box, key=lambda x: x[1])  # Sort by y-coordinate (top to bottom)
    top_left, top_right = sorted_box[:2]
    # Ensure top_left and top_right are tuples (x, y)
    top_left = tuple(top_left)
    top_right = tuple(top_right)

    # Draw circles at the top_left and top_right points
    cv2.circle(display_image, top_left, 2, (0, 255, 255), -1)
    cv2.circle(display_image, top_right, 2, (0, 255, 255), -1)

    # Calculate the direction vector of the top edge
    edge_vector = np.array([top_right[0] - top_left[0], top_right[1] - top_left[1]])
    edge_length = np.linalg.norm(edge_vector)
    if edge_length == 0:
        return None  # Avoid division by zero

    # Normalize the edge vector and compute the perpendicular vector
    edge_unit_vector = edge_vector / edge_length
    perpendicular_vector = np.array([-edge_unit_vector[1], edge_unit_vector[0]])

    # Calculate the midpoint of the top edge
    midpoint = ((top_left[0] + top_right[0]) // 2, (top_left[1] + top_right[1]) // 2)

    # Calculate the gradient of the edge (slope)
    gradient = edge_vector[1] / edge_vector[0] if edge_vector[0] != 0 else float('inf')

    # Check if the gradient is negative and adjust the perpendicular vector direction
    if gradient < 0:
        perpendicular_vector = -perpendicular_vector  # Reverse the perpendicular vector if gradient is negative

    # Calculate the center point to check above the midpoint by `check_distance`
    check_point = (
        int(midpoint[0] - perpendicular_vector[0] * check_distance),
        int(midpoint[1] - perpendicular_vector[1] * check_distance)
    )
    cv2.circle(display_image, check_point, region_size, (255, 0, 255), -1)

    # Define the region around the check point
    region_half = region_size // 2
    start_x = check_point[0] - region_half
    end_x = check_point[0] + region_half + 1
    start_y = check_point[1] - region_half
    end_y = check_point[1] + region_half + 1

    # Check if the region is within the image bounds
    if start_x < 0 or end_x > black_image.shape[1] or start_y < 0 or end_y > black_image.shape[0]:
        return None, display_image  # Region is out of bounds

    # Check if any pixel in the region is black
    if np.any(black_image[start_y:end_y, start_x:end_x] == 255):
        return box, display_image  # Return the 4 corners of the valid minAreaRect

    return None, display_image  # If no black in the region, return None

def validate_multiple_contours(contours, black_image, display_image):
    if len(contours) > 2:
        contours_sorted = sorted(contours, key=lambda cnt: cv2.boundingRect(cnt)[1])
        contours = contours_sorted[:2]

    valid_rects = []
    for contour in contours:
        valid_rect, display_image = validate_green_contour(contour, black_image, display_image)
        if valid_rect is not None:
            valid_rects.append(valid_rect)
    
    if len(valid_rects) == 2:
        return valid_rects
    elif len(valid_rects) == 1:
        _, y1, _, h1 = cv2.boundingRect(contours[0])
        _, y2, _, h2 = cv2.boundingRect(contours[1])
        y_difference = abs((y1 + h1)/2 - (y2 + h2)/2)
        # print(f"  y {y_difference}")

        if y_difference < 15:
            valid_rects = []
            for contour in contours:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)  # Get the 4 corners of the rectangle
                box = np.int0(box)
                valid_rects.append(box)
                
            # print(f"  rects {valid_rects}")
            return valid_rects, display_image
        else:
            valid_rect = []
            valid_rect.append(valid_rects[0])
            return  valid_rect, display_image
    else:
        return None, display_image

def green_sign(valid_rects, black_image, display_image):
    global green_turn
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
                cv2.putText(display_image, ">", (WIDTH - 55, HEIGHT // 2 + 25), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 0, 255), 2)
                green_turn = "Right"
                return "Right"
        # If no black on the left, check the bottom-right corner (left turn)
        right_check_x = right_bottom[0] + 10
        right_check_y = right_bottom[1]
        if 0 <= right_check_y < HEIGHT and 0 <= right_check_x < WIDTH:
            if black_image[right_check_y, right_check_x] == 255:
                cv2.putText(display_image, "<", (0, HEIGHT // 2 + 25), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 0, 255), 2)
                green_turn = "Left"
                return "Left"
            else:
                # Ignore if no black detected at either side
                green_turn = None
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
            cv2.putText(display_image, "U", (WIDTH // 2 - 23, HEIGHT // 2 + 35), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 255), 2)
            green_turn = "U-Turn"
            return "U-Turn"
        else:
            # Ignore if no black detected at the midpoint
            green_turn = None
            return "None"