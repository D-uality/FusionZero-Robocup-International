from core.shared_imports import cv2, np, Picamera2, Transform, os
from core.utilities import debug, user_at_host

os.environ["LIBCAMERA_LOG_LEVELS"] = "2"

class Camera():
    def __init__(self):
        # Camera settings   
        self.X11 = True
        self.FLIP = False
        self.debug = False
        self.LINE_WIDTH = 320
        self.LINE_HEIGHT = 200

        self.color_format = "RGB888"

        # Transformation points
        if user_at_host == "frederick@raspberrypi":
            self.top_left     = (int(self.LINE_WIDTH / 4),     int(self.LINE_HEIGHT / 10))
            self.top_right    = (int(self.LINE_WIDTH * 3 / 4), int(self.LINE_HEIGHT / 10))
            self.bottom_left  = (0,                            self.LINE_HEIGHT - int(self.LINE_HEIGHT / 4))
            self.bottom_right = (self.LINE_WIDTH,              self.LINE_HEIGHT - int(self.LINE_HEIGHT / 4))
        else:
            self.top_left     = (int(self.LINE_WIDTH / 4),     int(self.LINE_HEIGHT / 5))
            self.top_right    = (int(self.LINE_WIDTH * 3 / 4), int(self.LINE_HEIGHT / 5))
            self.bottom_left  = (0,                            self.LINE_HEIGHT - 70)   # self.LINE_HEIGHT - int(3*self.LINE_HEIGHT / 10)
            self.bottom_right = (self.LINE_WIDTH,              self.LINE_HEIGHT - 70)   # self.LINE_HEIGHT - int(3*self.LINE_HEIGHT / 10)

        top_left =     (40,   70)
        top_right =    (120,  70)
        bottom_left =  (45,   100)
        bottom_right = (125,  100)
        self.light_point_left = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32) # 50, 50, 150, 100
        
        top_left =     (195,  45)
        top_right =    (290,  45)
        bottom_left =  (195,  85)
        bottom_right = (280,  85)
        self.light_point_right = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32) # 50 200 70 290

        top_left =     (50,                     110)
        top_right =    (self.LINE_WIDTH - 40,   100)
        bottom_left =  (65,                     130)
        bottom_right = (self.LINE_WIDTH - 45,   120)
        self.light_point_mid_top = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32)

        top_left =     (70,                     140)
        top_right =    (self.LINE_WIDTH - 30,   125)
        bottom_left =  (80,                     160)
        bottom_right = (self.LINE_WIDTH - 40,   145)
        self.light_point_mid = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32)

        # top_left =     (70,                    self.LINE_HEIGHT - 20)
        # top_right =    (self.LINE_WIDTH - 45,  self.LINE_HEIGHT - 20)
        # bottom_left =  (80,                    self.LINE_HEIGHT)
        # bottom_right = (self.LINE_WIDTH - 55,  self.LINE_HEIGHT)
        # self.light_point_mid_bottom = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32)

        top_left =     (0,  self.LINE_HEIGHT - 70)
        bottom_left =  (0,  self.LINE_HEIGHT)
        bottom_right = (70, self.LINE_HEIGHT)
        self.dark_left = np.array([top_left, bottom_left, bottom_right], dtype=np.float32)
         
        top_right =    (self.LINE_WIDTH,        self.LINE_HEIGHT - 70)
        bottom_left =  (self.LINE_WIDTH,        self.LINE_HEIGHT)
        bottom_right = (self.LINE_WIDTH - 70,   self.LINE_HEIGHT)
        self.dark_right = np.array([top_right, bottom_left, bottom_right], dtype=np.float32)
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
            cv2.polylines(transformed_image, [np.int32(self.light_point_left)],         isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.polylines(transformed_image, [np.int32(self.light_point_right)],        isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.polylines(transformed_image, [np.int32(self.light_point_mid_top)],      isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.polylines(transformed_image, [np.int32(self.light_point_mid)],          isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.polylines(transformed_image, [np.int32(self.light_point_mid_bottom)],   isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.polylines(transformed_image, [np.int32(self.dark_left)],                isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.polylines(transformed_image, [np.int32(self.dark_right)],               isClosed=True, color=(0, 255, 0), thickness=2)

        return transformed_image
    
    def close(self):
        if self.camera:
            self.camera.close()
            self.camera = None
            debug(["TERMINATION", "CAMERA", "✓"], [25, 25, 50])
        else:
            debug(["TERMINATION", "CAMERA", "X"], [25, 25, 50])

        if self.X11: cv2.destroyAllWindows()