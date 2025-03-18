import cv2
import numpy as np
import time
from picamera2 import Picamera2
from libcamera import Transform  # Correctly import Transform class

# Config variables
WIDTH, HEIGHT = 640, 480  # Correct resolution for Pi Camera
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
top_left = (50, 230)
bottom_left = (0, HEIGHT)
top_right = (WIDTH-50, 230)
bottom_right = (WIDTH, HEIGHT)

def perspective_transform(image):
    """Apply perspective transform to the image"""
    # Define points for the perspective transform (source and destination)
    src_points = np.float32([top_left, bottom_left, top_right, bottom_right])
    dst_points = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

    # Calculate the perspective matrix
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Apply the perspective transform to the image
    transformed_image = cv2.warpPerspective(image, matrix, (image.shape[1], image.shape[0]))

    return transformed_image

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

        # Apply perspective transform
        transformed_image = perspective_transform(image)

        # Calculate FPS
        fps = 1 / (time.time() - time_start)

        # Display the original and transformed images
        cv2.imshow("Original Image", image)
        cv2.imshow("Transformed Image", transformed_image)
        print(f"FPS: {fps:.2f}")

        # Wait for key press to close window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting program!")

finally:
    camera.stop()
    cv2.destroyAllWindows()