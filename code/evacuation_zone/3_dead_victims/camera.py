import cv2
from picamera2 import Picamera2
from libcamera import Transform
import oled_display
import config
    
camera = None

def initialise():
    global camera
    try:
        camera = Picamera2()

        camera_config = camera.create_preview_configuration(main={"format": "RGB888", "size": (config.WIDTH, config.HEIGHT)}, transform=Transform(vflip=config.FLIP, hflip=config.FLIP))
        camera.configure(camera_config)
        camera.start()
        
        config.status_messages.append(["Camera", "✓"])
        oled_display.text("Camera: ✓", 0, 40)
    except Exception as e:
        print(f"Camera failed to initialise: {e}")
        oled_display.text("Camera: X", 0, 40)

    if config.X11:
        try:
            cv2.startWindowThread()
            config.status_messages.append(["X11", "✓"])
            oled_display.text("X11: ✓", 60, 40)
        except Exception as e:
            config.status_messages.append(["X11", "X", f"{e}"])
            oled_display.text("X11: X", 60, 40)

def close():
    global camera
    camera.close()

def capture_array():
    global camera
    return camera.capture_array()