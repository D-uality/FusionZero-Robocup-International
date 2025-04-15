import cv2
import numpy as np

from config import *
from green import *

min_black_area = 100
prev_angle = 90
gap_found = 0

black_threshold = ((0, 0, 0), (180, 255, 70))

def black_mask(image, display_image):
    global green_sign
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    black_mask = cv2.inRange(hsv_image, black_threshold[0], black_threshold[1])

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 
    black_mask = cv2.erode(black_mask, kernel, iterations=3)
    black_mask = cv2.dilate(black_mask, kernel, iterations=4)

    contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_black_area]

    if len(contours) == 1:
        contour = contours[0]
    elif len(contours) == 0:
        contour = None
    elif len(contours) > 1: 
        true_prev_angle = prev_angle
        angle_difference = []
        _ = None
        for contour in contours:
            angle, _ = calculate_angle(contour, _)
            angle_difference.append(abs(true_prev_angle - angle))
        
        # Find the index of the contour with the smallest angle difference
        min_angle_index = angle_difference.index(min(angle_difference))
        
        # Select the contour with the smallest angle difference
        contour = contours[min_angle_index]
        
    if contour is not None and display_image is not None:
        cv2.drawContours(display_image, contours, -1, (255, 0, 0), 2)

    return contour, black_mask

def calculate_bottom_points(contour):
    bottom_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][1] >= HEIGHT-1]

    bottom_edge_points.sort(key=lambda p: p[0])
    if len(bottom_edge_points) > 2:
        for i in range(1, len(bottom_edge_points)):
            prev_point = bottom_edge_points[i - 1]
            curr_point = bottom_edge_points[i]
            
            bottom = curr_point[0] - prev_point[0]
            
            if bottom >= 20:
                return prev_point, curr_point

    return None

def calculate_top_contour(contour):
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

    return ((top_left[0] + top_right[0]) // 2, (top_left[1] + top_right[1]) // 2)

def calculate_angle(contour, display_image):
    global prev_angle, gap_found, green_sign
    angle = 90

    if contour is not None:
        # Adjust edge definitions to include a few pixels margin
        top_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][1] <= 5]
        left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= 10]
        right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= WIDTH - 10]


        ref_point = None

        # Determine point based on specified rules
        if top_edge_points:
            x_avg = int(np.mean([p[0] for p in top_edge_points]))
            ref_point = (x_avg, 0)
        else:
            if green_sign is not None:
                if green_sign == "Left":
                    prev_angle = 0
                if green_sign == "Right":
                    prev_angle = 180
            bottom_points = calculate_bottom_points(contour)
            if bottom_points:
                leftmost_point, rightmost_point = bottom_points

                if angle < 90:  # Left side
                    # Calculate distance from the leftmost point to the center
                    distance = leftmost_point[0] - (WIDTH // 2)
                    angle = 90 + int(distance / WIDTH * 90)  # Return a negative value less than 90
                    if display_image is not None:
                        cv2.line(display_image, leftmost_point, (leftmost_point[0], 0), (0, 0, 255), 2)
                elif angle > 90:  # Right side
                    # Calculate distance from the rightmost point to the center
                    distance = rightmost_point[0] - (WIDTH // 2)
                    angle = 90 + int(distance / WIDTH * 90)  # Return a negative value less than 90
                    if display_image is not None:
                        cv2.line(display_image, rightmost_point, (rightmost_point[0], 0), (0, 0, 255), 2)
            
            elif left_edge_points and right_edge_points:
                if prev_angle < 90:
                    y_avg = int(np.mean([p[1] for p in left_edge_points]))
                    ref_point = (0, y_avg)
                elif prev_angle > 90:
                    y_avg = int(np.mean([p[1] for p in right_edge_points]))
                    ref_point = (WIDTH - 1, y_avg)
                else:
                    left_y_avg = int(np.mean([p[1] for p in left_edge_points]))
                    right_y_avg = int(np.mean([p[1] for p in right_edge_points]))
                    if left_y_avg < right_y_avg:
                        ref_point = (0, left_y_avg)
                    elif left_y_avg> right_y_avg:
                        ref_point = (WIDTH - 1, right_y_avg)
            elif left_edge_points:
                y_avg = int(np.mean([p[1] for p in left_edge_points]))
                ref_point = (0, y_avg)
            elif right_edge_points:
                y_avg = int(np.mean([p[1] for p in right_edge_points]))
                ref_point = (WIDTH, y_avg)
            else:
                ref_point = calculate_top_contour(contour)
                

        if ref_point:
            bottom_center = (WIDTH // 2, HEIGHT)
            dx = bottom_center[0] - ref_point[0]
            dy = bottom_center[1] - ref_point[1]

            # Calculate angle in degrees
            angle_radians = np.arctan2(dy, dx)
            angle = int(np.degrees(angle_radians))
            prev_angle = angle

            # Visualize the calculation
            if display_image is not None:
                cv2.line(display_image, ref_point, bottom_center, (0, 0, 255), 2)
            gap_found = 0
        else:
            gap_found += 1
    else:
        gap_found += 1

    return angle, gap_found