import config

import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform
# import oled_display

min_black_area = 1000
camera = None
camera_mode = ""

def initialise(mode: str):
    global camera, camera_mode
    try:
        camera = Picamera2()
        
        if "evac" in mode:
            camera_config = camera.create_still_configuration(
                main={"size": (config.EVACUATION_WIDTH, config.EVACUATION_HEIGHT), "format": "YUV420"},
                # raw={"size": (2304, 1296), "format": "SBGGR10"},
                raw={"size": (2304, 1500), "format": "SBGGR10"},
                transform=Transform(vflip=config.FLIP, hflip=config.FLIP)
            )
            camera_mode = "evac"
        else:
            camera_config = camera.create_still_configuration(
                main={"size": (config.LINE_WIDTH, config.LINE_HEIGHT), "format": "YUV420"},
                # raw={"size": (2304, 1296), "format": "SBGGR10"},
                raw={"size": (2304, 1500), "format": "SBGGR10"},
                transform=Transform(vflip=config.FLIP, hflip=config.FLIP)
            )
            controls = {
                # "ExposureTime": 10000,  # Shutter speed in microseconds
                "ExposureTime": 300,  # Shutter speed in microseconds
                "AnalogueGain": 0     # Analog gain (ISO-like setting)
            }

            # camera_config = camera.create_preview_configuration(main={"format": "RGB888", "size": (config.LINE_WIDTH, config.LINE_HEIGHT)})
            camera_mode = "line"
        
        camera.configure(camera_config)
        camera.set_controls(controls)
        camera.start()
        
        config.update_log(["INITIALISATION", "CAMERA", "✓"], [24, 15, 50])
        print()
        # oled_display.text("Camera: ✓", 0, 40)
        
    except Exception as e:
        config.update_log(["INITIALISATION", "CAMERA", f"{e}"], [24, 15, 50])
        print()
        # oled_display.text("Camera: X", 0, 40)
        raise e

    if config.X11:
        try:
            cv2.startWindowThread()
            
            config.update_log(["INITIALISATION", "X11", "✓"], [24, 15, 50])
            print()            
            # oled_display.text("X11: ✓", 60, 40)
            
        except Exception as e:
            config.update_log(["INITIALISATION", "X11", f"{e}"], [24, 15, 50])
            print()
            # oled_display.text("X11: X", 60, 40)
            raise e

def perspective_transform(image, mode):
    """Apply perspective transform to the image"""
    if "UPHILL" in mode or "DOWNHILL" in mode:
        view_multi = 1.2
    else:
        view_multi = 2.2

    top_left =      (int(config.LINE_WIDTH / 4), int(config.LINE_HEIGHT / 4.5))
    bottom_left =   (0, config.LINE_HEIGHT - 1)
    top_right =     (config.LINE_WIDTH - int(config.LINE_WIDTH / 4), int(config.LINE_HEIGHT / 4.5))
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
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    black_mask = cv2.inRange(image, (0, 0, 0), (255, 255, 40))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    black_mask = cv2.erode(black_mask, kernel, iterations=1)

    contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter by area threshold
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_black_area]

    # Find highest contour
    contour = None
    min_y = float('inf')
    for cnt in contours:
        highest_point = min(cnt, key=lambda pt: pt[0][1])
        if highest_point[0][1] < min_y:
            min_y = highest_point[0][1]
            contour = cnt

    # Draw all contours in blue, and the largest in cyan
    if display_image is not None:
        for c in contours:
            color = (255, 255, 0) if c is contour else (255, 0, 0)
            cv2.drawContours(display_image, [c], -1, color, 2)

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

def calculate_top_contour(contour, display_image=None, num_points=2):
    # Flatten the contour array to a list of (x, y) tuples
    points = [tuple(pt[0]) for pt in contour]

    # Sort by y-coordinate (top = smallest y)
    points.sort(key=lambda p: p[1])

    # Take the top N highest points (or all if fewer)
    top_points = points[:min(num_points, len(points))]

    if not top_points:
        return None  # No points to average
            
    # Draw each of the top points in red
    if display_image is not None:
        for point in top_points:
            cv2.circle(display_image, point, 3, (0, 0, 255), -1)  # Red dot, filled

    # Calculate average x and y
    x_avg = int(sum(p[0] for p in top_points) / len(top_points))
    y_avg = int(sum(p[1] for p in top_points) / len(top_points))

    return (x_avg, y_avg)

def calculate_angle(contour, display_image, prev_angle):
    angle = 90

    if contour is not None:
        ref_point = calculate_top_contour(contour, display_image)

        # Adjust edge definitions to include a few pixels margin
        left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= 10]
        right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= config.LINE_WIDTH - 10]

        # If the highest reference on the line is too low use left or right instead
        if ref_point[1] > 10 and (left_edge_points or right_edge_points):
            if left_edge_points and prev_angle < 90:
                y_avg = int(np.mean([p[1] for p in left_edge_points]))
                ref_point = (0, y_avg)
            elif right_edge_points and prev_angle > 90:
                y_avg = int(np.mean([p[1] for p in right_edge_points]))
                ref_point = (config.LINE_WIDTH - 1, y_avg)
            else:
                left_y_avg = right_y_avg = None

                if left_edge_points:
                    left_y_avg = int(np.mean([p[1] for p in left_edge_points]))
                    ref_point = (0, left_y_avg)
                if right_edge_points:
                    right_y_avg = int(np.mean([p[1] for p in right_edge_points]))
                    ref_point = (config.LINE_WIDTH - 5, right_y_avg)

                # Compare left and right averages if they are both true
                if left_y_avg and right_y_avg:
                    if left_y_avg < right_y_avg:
                        ref_point = (0, left_y_avg)
                    elif left_y_avg > right_y_avg:
                        ref_point = (config.LINE_WIDTH, right_y_avg)

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
    global camera, camera_mode
    image = camera.capture_array()
    
    # if camera_mode == "evac":
    image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
    # image = image[20:, :]
    # image = image[:, :]

    return image
