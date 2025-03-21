import cv2
import numpy as np
import time
from picamera2 import Picamera2
from libcamera import Transform  # Correctly import Transform class

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
bottom_left = (0, HEIGHT-1)
top_right = (WIDTH-10, 100)
bottom_right = (WIDTH, HEIGHT-1)

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

def black_mask(image):
    """Find the black line"""
    # Specific Y for black mask
    y_threshold = image[250:300, :]

    # Mask for black
    black_mask = cv2.inRange(y_threshold, (0, 0, 0), (80, 80, 80))

    kernel = np.ones((3,3), np.uint8)
    black_mask = cv2.erode(black_mask, kernel, iterations = 8)
    black_mask = cv2.dilate(black_mask, kernel, iterations = 9)

    contours, hierarchy = cv2.findContours(black_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    # cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

    if len(contours) > 0:
        # Draw Rectangle and line for visualizing x centre of line
        x, y, w, h = cv2.boundingRect(contours[0])
        # cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 3)
        cv2.line(image, (int(x+(w/2)), 250), (int(x+(w/2)), 300), (255, 0, 0), 3)

    return black_mask

try:
    while True:
        time_start = time.time()  # Start timing FPS calculation

        # Capture image
        image = camera.capture_array()

        # Crop image based on config
        image = image[CROP_MIN:CROP_MAX, :]

        # Draw Source Coordinates
        cv2.circle(image, top_left, 3, (0, 0, 255), -1)
        cv2.circle(image, bottom_left, 3, (0, 0, 255), -1)
        cv2.circle(image, top_right, 3, (0, 0, 255), -1)
        cv2.circle(image, bottom_right, 3, (0, 0, 255), -1)

        # Apply perspective transforme
        transformed_image = perspective_transform(image)

        # Find and black line
        black = black_mask(transformed_image)

        # Calculate FPS
        fps = 1 / (time.time() - time_start)

        # Display the original and transformed images
        cv2.imshow("Original Image", image)
        cv2.imshow("Transformed Image", transformed_image)
        # cv2.imshow("Black Mask", black)
        print(f"FPS: {fps:.2f}")

        # Wait for key press to close window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting program!")

finally:
    camera.stop()
    cv2.destroyAllWindows()