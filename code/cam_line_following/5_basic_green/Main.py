import cv2
import numpy as np
import time
from picamera2 import Picamera2
from libcamera import Transform

from config import *
from transform import *
from black import *
from green import *
from motor import *

motor_speed = (20, 15)
error_multi = 0.8

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

        # Find colours
        black_contour, black_image = black_mask(transformed_image, line_image)

        green_contours, green_image = green_mask(transformed_image, line_image)

        valid_corners = []
        for contour in green_contours:
            valid_rect = validate_green_contour(contour, black_image)
            if valid_rect is not None:
                valid_corners.append(valid_rect)

        green = green_sign(valid_corners, black_image, line_image)
        
        if green == "Left":
            servo_write(-motor_speed[0]-5, motor_speed[1]+5)
            time.sleep(0.5)
            servo_write(motor_speed[0]+5, motor_speed[1]+5)
            time.sleep(0.5)
        elif green == "Right":
            servo_write(motor_speed[0]+5, -motor_speed[1]-5)
            time.sleep(0.5)
            servo_write(motor_speed[0]+5, motor_speed[1]+5)
            time.sleep(0.5)
        elif green == "U-Turn":
            servo_write(motor_speed[0]+5, motor_speed[1]+5)
            time.sleep(0.8)
            servo_write(-motor_speed[0]-15, motor_speed[1]+15)
            time.sleep(1.5)
        else:
            angle = calculate_angle(black_contour, line_image)

            error = angle - 90
            turn = int(error * error_multi)

            servo_write(motor_speed[0] + turn , motor_speed[1] - turn)

        fps = int(1 / (time.time() - time_start))

        # Display images
        if X11 is True:
            cv2.imshow("Line", line_image)
            key = cv2.waitKey(1)
            if key == 27:  # Escape key to break the loop
                break

        print(f"FPS: {fps}   |   Turn: {turn},   Green: {green},   Error: {error},    Angle: {angle}")
        
except KeyboardInterrupt:
    print("Exiting program!")
finally:
    # Ensure cleanup happens whether exiting normally or with Ctrl+C
    servo_write(0, 0)
    if X11 is True:
        cv2.destroyAllWindows()
        camera.stop()