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
from led import *

motor_speed = (23, 15)
error_multi = 0.7
blink_delay = 0.2

angle, error, turn, green = 90, 0, 0, "None"
servo_write(0, 0)
led_on()

print("Press Enter to start...")
while True and X11 is True:
    image = camera.capture_array()
    cv2.imshow("Image", image)
    key = cv2.waitKey(1)
    if key == 13:
        cv2.destroyWindow("Image")
        break
print("Starting the try loop...")
led_blink(blink_delay)

try:
    if X11 is True: 
        motor_running = False
    else:
        motor_running = True
    while True:
        time_start = time.time()
        image = camera.capture_array()

        transformed_image = perspective_transform(image)
        line_image = transformed_image.copy()

        black_contour, black_image = black_mask(transformed_image, line_image)
        green_contours, green_image = green_mask(transformed_image, line_image)

        valid_corners = []
        if len(green_contours) == 1:
            valid_rect, line_image = validate_green_contour(green_contours[0], black_image, line_image)
            if valid_rect is not None:
                valid_corners.append(valid_rect)
        elif len(green_contours) > 1:
            valid_corners, line_image = validate_multiple_contours(green_contours, black_image, line_image)

        if valid_corners is not None:
            green = green_sign(valid_corners, black_image, line_image)
        else:
            green = "None"
        
        angle = calculate_angle(black_contour, line_image)

        if X11 is True:
            cv2.imshow("Line", line_image)
            key = cv2.waitKey(1)
            if key == 27:
                break
            elif key == ord('r'):
                motor_running = True
                led_blink(blink_delay)
            elif key != -1:
                motor_running = False
                servo_write(0, 0)
                led_blink(blink_delay)

        if green == "Left" and motor_running:
            angle = 90-50
        elif green == "Right" and motor_running:
            angle = 90+50
        elif green == "U-Turn" and motor_running:
            servo_write(motor_speed[0]+5, motor_speed[1]+5)
            time.sleep(0.8 - 2 * blink_delay)
            led_blink(blink_delay)
            servo_write(-motor_speed[0]-10, motor_speed[1]+10)
            time.sleep(1.5)

        error = angle - 90
        turn = int(error * error_multi)

        if motor_running:
            servo_write(motor_speed[0] + turn , motor_speed[1] - turn)

        fps = int(1 / (time.time() - time_start))
        print(f"FPS: {fps}   |   Turn: {turn},   Green: {green},   Error: {error},    Angle: {angle}")
        
except KeyboardInterrupt:
    print("Exiting program!")

led_blink(blink_delay)
servo_write(0, 0)
time.sleep(0.2)
led_close()
led_off()
if X11 is True:
    cv2.destroyAllWindows()
    camera.stop()