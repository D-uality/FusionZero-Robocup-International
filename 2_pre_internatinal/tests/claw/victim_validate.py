import time
from adafruit_servokit import ServoKit
import cv2
from picamera2 import Picamera2
from libcamera import Transform
import numpy as np

WIDTH, HEIGHT, FLIP = 320, 240, False
claw_pin = 9

camera = Picamera2()
camera_config = camera.create_preview_configuration(main={"format": "RGB888", "size": (WIDTH, HEIGHT)}, transform=Transform(vflip=FLIP, hflip=FLIP))
camera.configure(camera_config)
camera.start()

pca = ServoKit(channels=16)
pca.servo[claw_pin].actuation_range = 270
pca.servo[claw_pin].angle = 270

cv2.startWindowThread()

def claw_step(target_angle, time_delay):
    if target_angle == 270: target_angle = 269
    if target_angle == 0:   target_angle = 1
    current_angle = pca.servo[claw_pin].angle
    
    if current_angle == target_angle:
        return
    
    elif current_angle > target_angle:
        while current_angle > target_angle:
            current_angle -= 1
            pca.servo[claw_pin].angle = current_angle
            time.sleep(time_delay)
    
    else:
        while current_angle < target_angle:
            current_angle += 1
            pca.servo[claw_pin].angle = current_angle
            time.sleep(time_delay)

while True:
    image = camera.capture_array()
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # image = cv2.GaussianBlur(image, (15, 15), 0)
    blue = cv2.inRange(hsv_image, (100, 0, 0), (160, 255, 60))

    contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)

        x, y, w, h = cv2.boundingRect(largest_contour)
        print(f"{y + h/2}")


    cv2.imshow("image", image)
    cv2.imshow("hsv_image", hsv_image)
    cv2.imshow("blue", blue)

    claw_step(int(input("ANGLE: ")), 0)