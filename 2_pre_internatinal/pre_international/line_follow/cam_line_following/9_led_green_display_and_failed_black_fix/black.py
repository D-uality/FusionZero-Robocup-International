import cv2
import numpy as np

from config import *

min_black_area = int(0.0065104166 * WIDTH * HEIGHT)
prev_ref_point = int(WIDTH / 2), int(HEIGHT)

black_threshold = ((0, 0, 0), (180, 255, 80))

def black_mask(image, display_image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    black_mask = cv2.inRange(hsv_image, black_threshold[0], black_threshold[1])

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 
    black_mask = cv2.erode(black_mask, kernel, iterations=int(HEIGHT/30))
    black_mask = cv2.dilate(black_mask, kernel, iterations=int(HEIGHT/20))

    contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_black_area]

    if len(contours) == 1:
        contour = contours[0]
    elif len(contours) == 0:
        contour = None
    elif len(contours) > 1:
        # Select the highest contour/contours
        top_scores = [cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] for c in contours]
        max_top_score = min(top_scores)
        top_contours = [c for c, score in zip(contours, top_scores) if score == max_top_score]
        contour = top_contours[0]

    if contour is not None and display_image is not None:
        cv2.drawContours(display_image, contours, -1, (255, 0, 0), 2)

    return contour, black_mask

def calculate_bottom_gap(contour):
    bottom_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][1] >= HEIGHT-1]

    bottom_edge_points.sort(key=lambda p: p[0])
    if len(bottom_edge_points) > 2:
        for i in range(1, len(bottom_edge_points)):
            prev_point = bottom_edge_points[i - 1]
            curr_point = bottom_edge_points[i]
            
            gap = curr_point[0] - prev_point[0]
            
            if gap >= 20:
                return prev_point, curr_point

    return None

def calculate_angle(contour, display_image):
    global prev_ref_point
    angle = 90

    if contour is not None:
        # Adjust edge definitions to include a few pixels margin
        top_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][1] <= 10]
        left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= 5]
        right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= WIDTH - 5]


        ref_point = None

        # Determine point based on specified rules
        if top_edge_points:
            x_avg = int(np.mean([p[0] for p in top_edge_points]))
            ref_point = (x_avg, 0)
        else:
            gap_points = calculate_bottom_gap(contour)
            if gap_points:
                leftmost_point, rightmost_point = gap_points

                if prev_ref_point[0] < (WIDTH // 2):  # Left side
                    # Calculate distance from the leftmost point to the center
                    distance = leftmost_point[0] - (WIDTH // 2)
                    angle = 90 + int(distance / WIDTH * 90)  # Return a negative value less than 90
                    if display_image is not None:
                        cv2.line(display_image, leftmost_point, (leftmost_point[0], 0), (0, 0, 255), 2)
                else:  # Right side
                    # Calculate distance from the rightmost point to the center
                    distance = rightmost_point[0] - (WIDTH // 2)
                    angle = 90 + int(distance / WIDTH * 90)  # Return a negative value less than 90
                    if display_image is not None:
                        cv2.line(display_image, rightmost_point, (rightmost_point[0], 0), (0, 0, 255), 2)
            
            elif left_edge_points and right_edge_points:
                if prev_ref_point[0] < (WIDTH // 2):
                    y_avg = int(np.mean([p[1] for p in left_edge_points]))
                    ref_point = (0, y_avg)
                elif prev_ref_point[0] > (WIDTH // 2):
                    y_avg = int(np.mean([p[1] for p in right_edge_points]))
                    ref_point = (WIDTH - 1, y_avg)
            elif left_edge_points:
                y_avg = int(np.mean([p[1] for p in left_edge_points]))
                ref_point = (0, y_avg)
            elif right_edge_points:
                y_avg = int(np.mean([p[1] for p in right_edge_points]))
                ref_point = (WIDTH, y_avg)

        if ref_point:
            prev_ref_point = ref_point

            bottom_center = (WIDTH // 2, HEIGHT)
            dx = bottom_center[0] - ref_point[0]
            dy = bottom_center[1] - ref_point[1]

            # Calculate angle in degrees
            angle_radians = np.arctan2(dy, dx)
            angle = int(np.degrees(angle_radians))

            # Visualize the calculation
            if display_image is not None:
                cv2.line(display_image, ref_point, bottom_center, (0, 0, 255), 2)

    return angle