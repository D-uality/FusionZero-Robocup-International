import config

import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform
import oled_display
    
camera = None

def initialise():
    global camera
    try:
        camera = Picamera2()

        camera_config = camera.create_preview_configuration(main={"format": "RGB888", "size": (config.WIDTH, config.HEIGHT)}, transform=Transform(vflip=config.FLIP, hflip=config.FLIP))
        camera.configure(camera_config)
        camera.start()
        
        print(["Camera", "✓"])
        oled_display.text("Camera: ✓", 0, 40)
        
    except Exception as e:
        print(f"Camera failed to initialise: {e}")
        oled_display.text("Camera: X", 0, 40)

    if config.X11:
        try:
            cv2.startWindowThread()
            print(["X11", "✓"])
            oled_display.text("X11: ✓", 60, 40)
        except Exception as e:
            print(["X11", "X", f"{e}"])
            oled_display.text("X11: X", 60, 40)


def perspective_transform(image):
    """Apply perspective transform to the image"""
    top_left =      (int(config.WIDTH / 32), int(config.HEIGHT / 2.4))
    bottom_left =   (0, config.HEIGHT - 1)
    top_right =     (config.WIDTH - int(config.WIDTH / 32), int(config.HEIGHT / 2.4))
    bottom_right =  (config.WIDTH, config.HEIGHT- 1 )
    
    # Draw Source Coordinates
    # cv2.circle(image, top_left, 3, (0, 0, 255), -1)
    # cv2.circle(image, bottom_left, 3, (0, 0, 255), -1)
    # cv2.circle(image, top_right, 3, (0, 0, 255), -1)
    # cv2.circle(image, bottom_right, 3, (0, 0, 255), -1)

    # Define points for the perspective transform (source and destination)
    src_points = np.float32([top_left, bottom_left, top_right, bottom_right])
    dst_points = np.float32([[0, 0], [0, config.HEIGHT], [config.WIDTH, 0], [config.WIDTH, config.HEIGHT]])

    # Calculate the perspective matrix
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Apply the perspective transform to the image
    transformed_image = cv2.warpPerspective(image, matrix, (image.shape[1], image.shape[0]))

    return transformed_image

def close():
    global camera
    camera.close()

def capture_array():
    global camera
    return camera.capture_array()