# import cv2, sys, time, datetime, os
# import RPi.GPIO as GPIO

# GPIO.setmode(GPIO.BCM)

# BUTTON_PIN = 22
# DEVICE_PATH = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
# SAVE_DIR = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_images/images"

# GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# prev_pressed = (GPIO.input(BUTTON_PIN) == GPIO.LOW)

# def main() -> None:
#     global prev_pressed
    
#     camera = cv2.VideoCapture(DEVICE_PATH, cv2.CAP_V4L2)
#     if not camera.isOpened(): sys.exit("Cannot Open Camera!")

#     camera.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
#     camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#     camera.set(cv2.CAP_PROP_FPS, 30)
#     camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
#     camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

#     os.makedirs(SAVE_DIR, exist_ok=True)
#     file_count = len([f for f in os.listdir(SAVE_DIR) if os.path.isfile(os.path.join(SAVE_DIR, f))])
#     print(f"Existing images: {file_count}")

#     try:
#         while True:
#             t0 = time.perf_counter()
#             ok, image = camera.read()
#             if not ok: continue

#             image = image[:int(960/2) - 160][:]
#             image = cv2.flip(image, 0)
#             image = cv2.flip(image, 1)

#             cv2.imshow("image", image)
#             user_input = cv2.waitKey(1) & 0xFF
#             pressed = (GPIO.input(BUTTON_PIN) == GPIO.LOW)
            
#             if user_input == ord("q"): break
            
#             elif user_input == ord("c") or pressed != prev_pressed:
#                 prev_pressed = pressed
                
#                 time_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
#                 path = os.path.join(SAVE_DIR, f"{time_stamp}.jpg")
                
#                 cv2.imwrite(path, image)
                
#                 file_count += 1
#                 print(f"Captured {path}  |  total images: {file_count}")

#             print(f"{1 / (time.perf_counter() - t0):.2f}")
#     finally:
#         camera.release()
#         cv2.destroyAllWindows()

# if __name__ == "__main__": main()

import os
import sys
import time
import datetime
import cv2
import socket
import getpass
import numpy as np
from libcamera import Transform
from picamera2 import Picamera2
import RPi.GPIO as GPIO

# Configure logging level for libcamera
os.environ["LIBCAMERA_LOG_LEVELS"] = "2"

# GPIO settings for capture button
BUTTON_PIN = 22
username = getpass.getuser()
hostname = socket.gethostname()
user_at_host = f"{username}@{hostname}"
if user_at_host == "frederick@raspberrypi":
    SAVE_DIR = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_images/images"
else:
    SAVE_DIR = "/home/aidan/FusionZero-Robocup-International/5_ai_training_data/0_images/images"

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
prev_pressed = (GPIO.input(BUTTON_PIN) == GPIO.LOW)

from gpiozero import LED
from time import sleep

led = LED(13)

led.on()

class Camera:
    def __init__(self):
        # Camera settings
        self.X11 = True
        self.FLIP = False
        self.debug = False
        self.LINE_WIDTH = 160
        self.LINE_HEIGHT = 100
        self.color_format = "RGB888"

        username = getpass.getuser()
        hostname = socket.gethostname()
        self.user_at_host = f"{username}@{hostname}"

        # Default perspective transform points
        self.top_left     = (int(self.LINE_WIDTH / 4),     10)
        self.top_right    = (int(self.LINE_WIDTH * 3 / 4), 10)
        self.bottom_left  = (0,                            self.LINE_HEIGHT - 30)
        self.bottom_right = (self.LINE_WIDTH,              self.LINE_HEIGHT - 30)

        # Adjust for specific host if needed
        if self.user_at_host == "frederick@raspberrypi":
            self.top_left     = (int(self.LINE_WIDTH / 4),     40)
            self.top_right    = (int(self.LINE_WIDTH * 3 / 4), 40)
            self.bottom_left  = (0,                            self.LINE_HEIGHT - 30)
            self.bottom_right = (self.LINE_WIDTH,              self.LINE_HEIGHT - 30)

        # Define auxiliary points for debugging overlay
        top_left =     (60,                    0)
        top_right =    (self.LINE_WIDTH - 55,  0)
        bottom_left =  (60,                    int(self.LINE_HEIGHT / 2.8) - 1)
        bottom_right = (self.LINE_WIDTH - 55,  int(self.LINE_HEIGHT / 2.8) - 1)
        self.light_points = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32)

        top_left =     (60,                    int(self.LINE_HEIGHT / 2.8))
        top_right =    (self.LINE_WIDTH - 55,  int(self.LINE_HEIGHT / 2.8))
        bottom_left =  (40,                    self.LINE_HEIGHT - 20)
        bottom_right = (self.LINE_WIDTH - 50,   self.LINE_HEIGHT - 15)
        self.lightest_points = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32)

        # Initialize Picamera2
        self.camera = Picamera2()
        camera_config = self.camera.create_preview_configuration(
            main      = {"size": (self.LINE_WIDTH, self.LINE_HEIGHT), "format": self.color_format},
            raw       = {"size": (2304, 1296), "format": "SBGGR10"},
            transform = Transform(vflip=self.FLIP, hflip=self.FLIP),
        )
        self.camera.configure(camera_config)
        self.camera.start()

        print("Camera initialized")

    def capture_array(self) -> np.ndarray:
        return self.camera.capture_array()

    def perspective_transform(self, image: np.ndarray) -> np.ndarray:
        src = np.array([self.top_left, self.top_right, self.bottom_left, self.bottom_right], dtype=np.float32)
        dst = np.array([[0, 0], [self.LINE_WIDTH, 0], [0, self.LINE_HEIGHT], [self.LINE_WIDTH, self.LINE_HEIGHT]], dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(image, matrix, (self.LINE_WIDTH, self.LINE_HEIGHT))

        if self.X11 and self.debug:
            cv2.polylines(warped, [np.int32(self.lightest_points)], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.polylines(warped, [np.int32(self.light_points)], isClosed=True, color=(0, 255, 0), thickness=2)
        return warped

    def close(self):
        if self.camera:
            self.camera.close()
            self.camera = None
            print("Camera closed")
        else:
            print("Camera was already closed")
        if self.X11:
            cv2.destroyAllWindows()


def main() -> None:
    global prev_pressed
    cam = Camera()
    os.makedirs(SAVE_DIR, exist_ok=True)
    file_count = len([f for f in os.listdir(SAVE_DIR) if os.path.isfile(os.path.join(SAVE_DIR, f))])
    print(f"Existing images: {file_count}")

    try:
        while True:
            t0 = time.perf_counter()
            frame = cam.capture_array()
            image = cam.perspective_transform(frame)

            cv2.imshow("image", image)
            key = cv2.waitKey(1) & 0xFF
            pressed = (GPIO.input(BUTTON_PIN) == GPIO.LOW)

            if key == ord('q'):
                break
            if key == ord('c') or pressed != prev_pressed:
                prev_pressed = pressed
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                path = os.path.join(SAVE_DIR, f"{timestamp}.jpg")
                cv2.imwrite(path, image)
                file_count += 1
                print(f"Captured {path}  |  total images: {file_count}")

            fps = 1.0 / (time.perf_counter() - t0)
            print(f"{fps:.1f} FPS   |   total images: {file_count}")
    finally:
        cam.close()


if __name__ == "__main__":
    main()