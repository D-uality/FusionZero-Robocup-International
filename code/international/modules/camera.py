import cv2

import numpy as np
from picamera2 import Picamera2
from libcamera import Transform
from utils import debug

class cCAMERA():
    def __init__(self, mode: str):
        # Camera settings
        self.X11 = True
        self.FLIP = False
        self.LINE_WIDTH = 320
        self.LINE_HEIGHT = 200
        self.EVAC_WIDTH = 320
        self.EVAC_HEIGHT = 200

        self.camera = None
        self.camera_mode = "line"
    
    def initialise(self, mode: str):
        try:
            self.camera = Picamera2()
            
            if "evac" in mode:
                camera_config = self.camera.create_still_configuration(
                    main={"size": (self.EVAC_WIDTH, self.EVAC_HEIGHT), "format": "YUV420"},
                    raw={"size": (2304, 1500), "format": "SBGGR10"},
                    transform=Transform(vflip=self.FLIP, hflip=self.FLIP)
                )
                self.camera_mode = "evac"
            else:
                camera_config = self.camera.create_still_configuration(
                    main={"size": (self.LINE_WIDTH, self.LINE_HEIGHT), "format": "YUV420"},
                    raw={"size": (2304, 1500), "format": "SBGGR10"},
                    transform=Transform(vflip=self.FLIP, hflip=self.FLIP)
                )
                controls = {
                    "ExposureTime": 300,
                    "AnalogueGain": 0
                }
                self.camera_mode = "line"
            
            self.camera.configure(camera_config)
            self.camera.set_controls(controls)
            self.camera.start()
            
            debug(["INITIALISATION", "CAMERA", "✓"], [24, 15, 50])
            print()
            # oled_display.reset()
            # oled_display.text("Camera: ✓", 0, 40)
            
        except Exception as e:
            debug(["INITIALISATION", "CAMERA", f"{e}"], [24, 15, 50])
            print()
            # oled_display.reset()
            # oled_display.text("Camera: X", 0, 40)
            raise e
    
    def capture_array(self) -> np.ndarray:
        image = self.camera.capture_array()
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)

        if self.camera_mode == "line":
            image = self.perspective_transform(image)
        
        return image

    def perspective_transform(self, image: np.ndarray) -> np.ndarray:

        # Transformation points
        top_left = (int(self.LINE_WIDTH / 4),      int(self.LINE_HEIGHT / 4))
        top_right = (int(self.LINE_WIDTH * 3 / 4), int(self.LINE_HEIGHT / 4))
        bottom_left = (0,                          self.LINE_HEIGHT - 1)
        bottom_right = (self.LINE_WIDTH,           self.LINE_HEIGHT - 1)

        src_points = np.array([top_left, top_right, bottom_left, bottom_right], dtype=np.float32)
        dst_points = np.array([[0, 0], [self.LINE_WIDTH, 0], [0, self.LINE_HEIGHT], [self.LINE_WIDTH, self.LINE_HEIGHT]], dtype=np.float32)
        
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        transformed_image = cv2.warpPerspective(image, matrix, (self.LINE_WIDTH, self.LINE_HEIGHT))

        return transformed_image
    
    def close(self):
        if self.camera:
            self.camera.close()
            self.camera = None
            debug(["TERMINATION", "CAMERA", "✓"], [24, 15, 50])
            print()
            # oled_display.reset()
            # oled_display.text("Camera: ✓", 0, 40)
            # oled_display.show()
        else:
            debug(["TERMINATION", "CAMERA", "X"], [24, 15, 50])
            print()
            # oled_display.reset()
            # oled_display.text("Camera: X", 0, 40)
            # oled_display.show()
        if self.X11:
            cv2.destroyAllWindows()