from core.shared_imports import cv2, np, Picamera2, Transform, os
from core.utilities import debug, user_at_host

os.environ["LIBCAMERA_LOG_LEVELS"] = "2"

class Camera():
    def __init__(self):
        # Camera settings   
        self.X11 = True
        self.FLIP = False
        self.debug = True
        self.LINE_WIDTH = 160
        self.LINE_HEIGHT = 100

        self.color_format = "RGB888"

        # Transformation points
        if user_at_host == "frederick@raspberrypi":
            self.top_left     = (int(self.LINE_WIDTH / 4),     20)
            self.top_right    = (int(self.LINE_WIDTH * 3 / 4), 20)
            self.bottom_left  = (0,                            self.LINE_HEIGHT - 15)
            self.bottom_right = (self.LINE_WIDTH,              self.LINE_HEIGHT - 15)
        else:
            self.top_left     = (int(self.LINE_WIDTH / 4),     10)
            self.top_right    = (int(self.LINE_WIDTH * 3 / 4), 10)
            self.bottom_left  = (0,                            self.LINE_HEIGHT - 30)
            self.bottom_right = (self.LINE_WIDTH,              self.LINE_HEIGHT - 30)
            
        top_left =     (30,                    0)
        top_right =    (self.LINE_WIDTH - 30,  0)
        bottom_left =  (30,                    int(self.LINE_HEIGHT / 2.8) - 1)
        bottom_right = (self.LINE_WIDTH - 30,  int(self.LINE_HEIGHT / 2.8) - 1)
        self.light_points = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32)

        top_left =     (30,                    int(self.LINE_HEIGHT / 2.8))
        top_right =    (self.LINE_WIDTH - 30,  int(self.LINE_HEIGHT / 2.8))
        bottom_left =  (20,                    self.LINE_HEIGHT - 10)
        bottom_right = (self.LINE_WIDTH - 20,    self.LINE_HEIGHT - 10)
        self.lightest_points = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32)
        self.camera = Picamera2()

        camera_config = self.camera.create_preview_configuration(
            main      = {"size": (self.LINE_WIDTH, self.LINE_HEIGHT), "format": self.color_format},
            raw       = {"size": (2304, 1296), "format": "SBGGR10"},
            transform = Transform(vflip=self.FLIP, hflip=self.FLIP),
        )

        self.camera.configure(camera_config)
        self.camera.start()
        
        debug(["INITIALISATION", "CAMERA", "✓"], [25, 25, 50])
            
    def capture_array(self) -> np.ndarray:
        return self.camera.capture_array()

    def perspective_transform(self, image: np.ndarray) -> np.ndarray:
        src_points = np.array([self.top_left, self.top_right, self.bottom_left, self.bottom_right], dtype=np.float32)
        dst_points = np.array([[0, 0], [self.LINE_WIDTH, 0], [0, self.LINE_HEIGHT], [self.LINE_WIDTH, self.LINE_HEIGHT]], dtype=np.float32)
        
        matrix            = cv2.getPerspectiveTransform(src_points, dst_points)
        transformed_image = cv2.warpPerspective(image, matrix, (self.LINE_WIDTH, self.LINE_HEIGHT))

        if self.X11 and self.debug:
            cv2.polylines(transformed_image, [np.int32(self.lightest_points)], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.polylines(transformed_image, [np.int32(self.light_points)], isClosed=True, color=(0, 255, 0), thickness=2)

        return transformed_image
    
    def close(self):
        if self.camera:
            self.camera.close()
            self.camera = None
            debug(["TERMINATION", "CAMERA", "✓"], [25, 25, 50])
        else:
            debug(["TERMINATION", "CAMERA", "X"], [25, 25, 50])

        if self.X11: cv2.destroyAllWindows()