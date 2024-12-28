import logging
import config

import cv2
from picamera2 import Picamera2
from libcamera import Transform
import oled_display

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
camera = None

def initialise():
    global camera
    try:
        camera = Picamera2()

        camera_config = camera.create_preview_configuration(main={"format": "RGB888", "size": (config.WIDTH, config.HEIGHT)}, transform=Transform(vflip=config.FLIP, hflip=config.FLIP))
        camera.configure(camera_config)
        camera.start()
        
        logging.info("Camera initialised!")
        oled_display.text("Camera: ✓", 0, 40)
    except Exception as e:
        logging.error(f"Camera failed to initialise: {e}")
        oled_display.text("Camera: X", 0, 40)

    if config.X11:
        try:
            cv2.startWindowThread()
            logging.info("X11 initialised!")
            oled_display.text("X11: ✓", 60, 40)
        except Exception as e:
            logging.error(f"X11 failed to initialise: {e}")
            oled_display.text("X11: X", 60, 40)

def close():
    global camera
    camera.close()

def capture_array():
    global camera
    return camera.capture_array()