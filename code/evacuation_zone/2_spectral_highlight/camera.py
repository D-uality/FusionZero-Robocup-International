from config import *

import cv2
from picamera2 import Picamera2
from libcamera import Transform
import oled_display

def initialise():
    try:
        camera = Picamera2()

        camera_config = camera.create_preview_configuration(main={"format": "RGB888", "size": (WIDTH, HEIGHT)}, transform=Transform(vflip=FLIP, hflip=FLIP))
        camera.configure(camera_config)
        camera.start()
        
        print("Camera initialised!")
        oled_display.text("Camera: ✓", 0, 40)

        if X11: 
            cv2.startWindowThread()
            oled_display.text("X11: ✓", 60, 40)

    except Exception as e:
        print(f"Camera failed to initialise: {e}")
        oled_display.text("Camera: X", 0, 40)

def close():
    camera.close()