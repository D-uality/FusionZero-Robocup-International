import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

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
from hardware.robot import *
from core.utilities import *

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

start_display()
led.on()

def main() -> None:
    global prev_pressed
    os.makedirs(SAVE_DIR, exist_ok=True)
    file_count = len([f for f in os.listdir(SAVE_DIR) if os.path.isfile(os.path.join(SAVE_DIR, f))])
    print(f"Existing images: {file_count}")

    try:
        while True:
            t0 = time.perf_counter()
            frame = camera.capture_array()
            image = camera.perspective_transform(frame)

            show(image, "image")
            pressed = (GPIO.input(BUTTON_PIN) == GPIO.LOW)

            if pressed != prev_pressed:
                prev_pressed = pressed
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                path = os.path.join(SAVE_DIR, f"{timestamp}.jpg")
                cv2.imwrite(path, image)
                file_count += 1
                print(f"Captured {path}  |  total images: {file_count}")

            fps = 1.0 / (time.perf_counter() - t0)
            print(f"{fps:.1f} FPS   |   total images: {file_count}")
    finally:
        led.off()
        GPIO.cleanup()
        print("LED's Off")
        stop_display()
        print("Display Stopped")
        camera.close()
        print("Camera Stopped")


if __name__ == "__main__":
    main()