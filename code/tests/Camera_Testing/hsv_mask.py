import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform

# Config variables
WIDTH, HEIGHT = 320, 240  # Correct resolution for Pi Camera
FLIP = False
CROP_MIN, CROP_MAX = 0, HEIGHT

# Initialize Picamera2
camera = Picamera2()

# Create a correct transform object from libcamera.Transform
camera_config = camera.create_preview_configuration(main={"format": "RGB888", "size": (WIDTH, HEIGHT)}, transform=Transform(vflip=FLIP, hflip=FLIP))

# Configure and start the camera
camera.configure(camera_config)
camera.start()

# Source Points
top_left = (10, 100)
bottom_left = (0, HEIGHT - 1)
top_right = (WIDTH - 10, 100)
bottom_right = (WIDTH, HEIGHT - 1)

# Initialize HSV thresholds
h_min, h_max = 0, 179
s_min, s_max = 0, 255
v_min, v_max = 0, 255

# Threshold selector
selectors = ["H_min", "H_max", "S_min", "S_max", "V_min", "V_max"]
current_selector = 0


def perspective_transform(image):
    """Apply perspective transform to the image"""
    # Define points for the perspective transform (source and destination)
    src_points = np.float32([top_left, bottom_left, top_right, bottom_right])
    dst_points = np.float32([[0, 0], [0, HEIGHT], [WIDTH, 0], [WIDTH, HEIGHT]])

    # Calculate the perspective matrix
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Apply the perspective transform to the image
    transformed_image = cv2.warpPerspective(image, matrix, (image.shape[1], image.shape[0]))

    return transformed_image


def hsv_mask(frame):
    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define HSV range and create mask
    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    # Apply mask to the frame
    result = cv2.bitwise_and(frame, frame, mask=mask)
    return mask, result


try:
    while True:
        # Capture image
        image = camera.capture_array()

        # Apply perspective transform
        transformed_image = perspective_transform(image)

        # Apply HSV mask
        masked_image, mask_result = hsv_mask(transformed_image)

        # Display the frames
        cv2.imshow("Original Frame", transformed_image)
        cv2.imshow("HSV Mask", masked_image)
        cv2.imshow("Result", mask_result)

        # Display current thresholds and selector
        print(f"Adjusting: {selectors[current_selector]} | H_min={h_min}, H_max={h_max}, S_min={s_min}, S_max={s_max}, V_min={v_min}, V_max={v_max}")

        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Quit
            break
        elif key == 82:  # Up arrow
            current_selector = (current_selector - 1) % len(selectors)
        elif key == 84:  # Down arrow
            current_selector = (current_selector + 1) % len(selectors)
        elif key == 81:  # Left arrow
            if selectors[current_selector] == "H_min":
                h_min = max(0, h_min - 1)
            elif selectors[current_selector] == "H_max":
                h_max = max(h_min, h_max - 1)
            elif selectors[current_selector] == "S_min":
                s_min = max(0, s_min - 1)
            elif selectors[current_selector] == "S_max":
                s_max = max(s_min, s_max - 1)
            elif selectors[current_selector] == "V_min":
                v_min = max(0, v_min - 1)
            elif selectors[current_selector] == "V_max":
                v_max = max(v_min, v_max - 1)
        elif key == 83:  # Right arrow
            if selectors[current_selector] == "H_min":
                h_min = min(h_max, h_min + 1)
            elif selectors[current_selector] == "H_max":
                h_max = min(179, h_max + 1)
            elif selectors[current_selector] == "S_min":
                s_min = min(s_max, s_min + 1)
            elif selectors[current_selector] == "S_max":
                s_max = min(255, s_max + 1)
            elif selectors[current_selector] == "V_min":
                v_min = min(v_max, v_min + 1)
            elif selectors[current_selector] == "V_max":
                v_max = min(255, v_max + 1)

except KeyboardInterrupt:
    print("Exiting loop...")

# Release the camera and close all OpenCV windows
cv2.destroyAllWindows()
camera.stop()
