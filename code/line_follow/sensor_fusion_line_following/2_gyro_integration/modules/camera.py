import config

import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform
import oled_display
    
min_black_area = 50
camera = None

def initialise(WIDTH, HEIGHT):
    global camera
    try:
        camera = Picamera2()
        camera_config = camera.create_still_configuration(
            main={"size": (WIDTH, HEIGHT), "format": "YUV420"},
            # raw={"size": (2304, 1296), "format": "SBGGR10"},
            raw={"size": (2304, 1500), "format": "SBGGR10"},
            transform=Transform(vflip=config.FLIP, hflip=config.FLIP)
        )
        camera.configure(camera_config)
        camera.start()
        
        config.update_log(["INITIALISATION", "CAMERA", "✓"], [24, 24, 3])
        oled_display.text("Camera: ✓", 0, 40)
        
    except Exception as e:
        config.update_log(["INITIALISATION", "CAMERA", "X"], [24, 24, 3])
        print(f"Camera failed to initialise: {e}")
        oled_display.text("Camera: X", 0, 40)
        exit()

    if config.X11:
        try:
            cv2.startWindowThread()
            config.update_log(["INITIALISATION", "X11", "✓"], [24, 24, 3])
            oled_display.text("X11: ✓", 60, 40)
        except Exception as e:
            config.update_log(["INITIALISATION", "X11", "X"], [24, 24, 3])
            print(f"X11 failed to initialise: {e}")
            oled_display.text("X11: X", 60, 40)
            exit()

def perspective_transform(image):
    """Apply perspective transform to the image"""
    top_left =      (int(config.LINE_WIDTH / 32), int(config.LINE_HEIGHT / 2.4))
    bottom_left =   (0, config.LINE_HEIGHT - 1)
    top_right =     (config.LINE_WIDTH - int(config.LINE_WIDTH / 32), int(config.LINE_HEIGHT / 2.4))
    bottom_right =  (config.LINE_WIDTH, config.LINE_HEIGHT- 1 )
    
    # Define points for the perspective transform (source and destination)
    src_points = np.float32([top_left, bottom_left, top_right, bottom_right])
    dst_points = np.float32([[0, 0], [0, config.LINE_HEIGHT], [config.LINE_WIDTH, 0], [config.LINE_WIDTH, config.LINE_HEIGHT]])

    # Calculate the perspective matrix
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Apply the perspective transform to the image
    transformed_image = cv2.warpPerspective(image, matrix, (image.shape[1], image.shape[0]))

    return transformed_image

def find_line_black_mask(image, display_image, prev_angle):
    black_mask = cv2.inRange(image, (0, 0, 0), (40, 40, 40))
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

        if true_prev_angle < 90:
            for contour in contours:
                angle = calculate_angle(contour, _, prev_angle)
                angle_difference.append(angle - true_prev_angle)
        elif true_prev_angle > 90:
            for contour in contours:
                angle = calculate_angle(contour, _, prev_angle)
                angle_difference.append(true_prev_angle - angle)
        else: 
            for contour in contours:
                angle = calculate_angle(contour, _, prev_angle)
                angle_difference.append(abs(true_prev_angle - angle))
            
        # Find the index of the contour with the smallest angle difference
        min_angle_index = angle_difference.index(min(angle_difference))
        
        # Select the contour with the smallest angle difference, and set the correct prev_angle
        contour = contours[min_angle_index]
        calculate_angle(contour, _, prev_angle)
        
    if contour is not None and display_image is not None:
        cv2.drawContours(display_image, contours, -1, (255, 0, 0), 2)

    return contour, black_mask

def calculate_bottom_points(contour):
    bottom_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][1] >= config.LINE_HEIGHT-1]

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

def calculate_angle(contour, display_image, prev_angle):
    angle = 90

    if contour is not None:
        # Adjust edge definitions to include a few pixels margin
        top_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][1] <= 5]
        left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= 10]
        right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= config.LINE_WIDTH - 10]

        ref_point = None

        # Determine point based on specified rules
        if top_edge_points:
            x_avg = int(np.mean([p[0] for p in top_edge_points]))
            ref_point = (x_avg, 0)
        else:
            bottom_points = calculate_bottom_points(contour)
            if bottom_points:
                leftmost_point, rightmost_point = bottom_points

                if angle < 90:  # Left side
                    # Calculate distance from the leftmost point to the center
                    distance = leftmost_point[0] - (config.LINE_WIDTH // 2)
                    angle = 90 + int(distance / config.LINE_WIDTH * 90)  # Return a negative value less than 90
                    if display_image is not None:
                        cv2.line(display_image, leftmost_point, (leftmost_point[0], 0), (0, 0, 255), 2)
                elif angle > 90:  # Right side
                    # Calculate distance from the rightmost point to the center
                    distance = rightmost_point[0] - (config.LINE_WIDTH // 2)
                    angle = 90 + int(distance / config.LINE_WIDTH * 90)  # Return a negative value less than 90
                    if display_image is not None:
                        cv2.line(display_image, rightmost_point, (rightmost_point[0], 0), (0, 0, 255), 2)
            
            elif left_edge_points and right_edge_points:
                if prev_angle < 90:
                    y_avg = int(np.mean([p[1] for p in left_edge_points]))
                    ref_point = (0, y_avg)
                elif prev_angle > 90:
                    y_avg = int(np.mean([p[1] for p in right_edge_points]))
                    ref_point = (config.LINE_WIDTH - 1, y_avg)
                else:
                    left_y_avg = int(np.mean([p[1] for p in left_edge_points]))
                    right_y_avg = int(np.mean([p[1] for p in right_edge_points]))
                    if left_y_avg < right_y_avg:
                        ref_point = (0, left_y_avg)
                    elif left_y_avg > right_y_avg:
                        ref_point = (config.LINE_WIDTH - 1, right_y_avg)
            else:
                top_ref_point = calculate_top_contour(contour)
                if top_ref_point[1] < config.LINE_HEIGHT/2:
                    ref_point = top_ref_point
                elif left_edge_points:
                    y_avg = int(np.mean([p[1] for p in left_edge_points]))
                    ref_point = (0, y_avg)
                elif right_edge_points:
                    y_avg = int(np.mean([p[1] for p in right_edge_points]))
                    ref_point = (config.LINE_WIDTH, y_avg)

        if ref_point:
            bottom_center = (config.LINE_WIDTH // 2, config.LINE_HEIGHT)
            dx = bottom_center[0] - ref_point[0]
            dy = bottom_center[1] - ref_point[1]

            # Calculate angle in degrees
            angle_radians = np.arctan2(dy, dx)
            angle = int(np.degrees(angle_radians))
            prev_angle = angle

            # Visualize the calculation
            if display_image is not None:
                cv2.line(display_image, ref_point, bottom_center, (0, 0, 255), 2)

    return angle

def close():
    global camera
    camera.close()

def capture_array():
    global camera
    
    image = camera.capture_array()
    image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
    image = image[20:, :]

    return image
