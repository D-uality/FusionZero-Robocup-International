import cv2
import numpy as np
import time
from picamera2 import Picamera2
from libcamera import Transform

from config import *
from transform import *
from line import *
from motor import *

motor_speed = (20, 15)
angle_multi = 0.5

servo_write(0, 0)

print("Press Enter to start...")
while True and X11 is True:
    image = camera.capture_array()
    cv2.imshow("Image", image)
    key = cv2.waitKey(1)
    if key == 13:
        cv2.destroyWindow("Image")
        break
print("Starting the try loop...")

try:
    while True:
        time_start = time.time()  # Start timing FPS 
        image = camera.capture_array()

        # Apply perspective transform
        transformed_image = perspective_transform(image)
        line_image = transformed_image.copy()

        # Find black line
        black_contour, black_image = black_mask(transformed_image, line_image)

        angle = calculate_angle(black_contour, line_image)

        turn = (angle - 90) * angle_multi
        servo_write(motor_speed[0] + turn , motor_speed[1] - turn)

        fps = int(1 / (time.time() - time_start))

        # Display images
        if X11 is True:
            cv2.imshow("Line Image", line_image)
            key = cv2.waitKey(1)
            if key == 27:  # Escape key to break the loop
                break

        print(f"FPS: {fps}   |   Angle: {angle}")
        
except KeyboardInterrupt:
    print("Exiting program!")
finally:
    # Ensure cleanup happens whether exiting normally or with Ctrl+C
    servo_write(0, 0)
    if X11 is True:
        cv2.destroyAllWindows()
        camera.stop()