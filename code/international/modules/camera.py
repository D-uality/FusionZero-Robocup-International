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
        self.color_format = "RGB888"

        self.camera = None
        self.camera_mode = "line"
    
    def initialise(self, mode: str):
        try:
            self.camera = Picamera2()
            
            if "evac" in mode:
                camera_config = self.camera.create_still_configuration(
                    main={"size": (self.EVAC_WIDTH, self.EVAC_HEIGHT), "format": self.color_format},
                    raw={"size": (2304, 1296), "format": "SBGGR10"},
                    transform=Transform(vflip=self.FLIP, hflip=self.FLIP)
                )
                self.camera_mode = "evac"
            else:
                camera_config = self.camera.create_preview_configuration(
                    main={"size": (self.LINE_WIDTH, self.LINE_HEIGHT), "format": self.color_format},
                    raw={"size": (2304, 1296), "format": "SBGGR10"},
                    transform=Transform(vflip=self.FLIP, hflip=self.FLIP),
                )
                controls = {
                    "ExposureTime": 100,
                    "AnalogueGain": 0
                }
                self.camera_mode = "line"
            
            self.camera.configure(camera_config)
            # self.camera.set_controls(controls)
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

        if self.camera_mode == "line":
            image = self.perspective_transform(image)
            # image = self.apply_reverse_vignette(image)

        return image


    def apply_reverse_vignette(self, img: np.ndarray) -> np.ndarray:
        rows, cols = img.shape[:2]
        y_indices, x_indices = np.indices((rows, cols))

        # 1️⃣ ellipse dimming (move it higher)
        center_x, center_y = cols / 2, rows * 0.55  # moved ellipse up
        stretch_x = 1.4  # slightly tighter width to avoid bleeding into side edges
        stretch_y = 1.3

        norm_x = (x_indices - center_x) / (cols / 2 / stretch_x)
        norm_y = (y_indices - center_y) / (rows / 2 / stretch_y)
        ellipse_r = np.sqrt(norm_x ** 2 + norm_y ** 2)
        ellipse_r = np.clip(ellipse_r, 0, 1)

        ellipse_dimming = 0.5 + (ellipse_r ** 2.2) * 0.5 # 0.6 center → 1.0 edge

        # 2️⃣ Corner Boosting (shrink effect from mid-left/mid-right)
        dist_tl = np.sqrt((x_indices)**2 + (y_indices)**2)
        dist_tr = np.sqrt((cols - x_indices)**2 + (y_indices)**2)
        dist_bl = np.sqrt((x_indices)**2 + (rows - y_indices)**2)
        dist_br = np.sqrt((cols - x_indices)**2 + (rows - y_indices)**2)
        corner_dist = np.minimum.reduce([dist_tl, dist_tr, dist_bl, dist_br])

        max_corner_dist = np.sqrt(cols**2 + rows**2)
        norm_corner = 1.0 - corner_dist / (0.45 * max_corner_dist)  # ← tighter spread
        norm_corner = np.clip(norm_corner, 0, 1)
        corner_boost = 1.0 + (norm_corner ** 4.0) * 2.5  # stronger, but more focused in corners

        # 3️⃣ Top and Bottom band boost (make flatter and stronger)
        norm_y_pos = y_indices / rows
        band_profile = np.cos(norm_y_pos * np.pi)  # peak = top/bottom
        band_boost = 1.0 + (band_profile ** 12) * 0.3  # softer than before


        # 4️⃣ Combine
        final_mask = ellipse_dimming * corner_boost * band_boost

        # 🧪 Debug visualisation
        mask_vis = final_mask.copy()
        if mask_vis.ndim == 3:
            mask_vis = mask_vis[:, :, 0]
        norm_vis = cv2.normalize(mask_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        # if self.X11:
        #     cv2.imshow("Vignette Mask (Grayscale)", norm_vis)
        #     cv2.waitKey(1)

        # ✅ Apply
        if img.ndim == 3:
            final_mask = np.repeat(final_mask[:, :, np.newaxis], 3, axis=2)

        result = img.astype(np.float32) * final_mask
        return np.clip(result, 0, 255).astype(np.uint8)

    def perspective_transform(self, image: np.ndarray) -> np.ndarray:

        # Transformation points
        top_left = (int(self.LINE_WIDTH / 4),      int(self.LINE_HEIGHT / 5))
        top_right = (int(self.LINE_WIDTH * 3 / 4), int(self.LINE_HEIGHT / 5))
        bottom_left = (0,                          self.LINE_HEIGHT - 1)
        bottom_right = (self.LINE_WIDTH,           self.LINE_HEIGHT - 1)

        src_points = np.array([top_left, top_right, bottom_left, bottom_right], dtype=np.float32)
        dst_points = np.array([[0, 0], [self.LINE_WIDTH, 0], [0, self.LINE_HEIGHT], [self.LINE_WIDTH, self.LINE_HEIGHT]], dtype=np.float32)
        
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        transformed_image = cv2.warpPerspective(image, matrix, (self.LINE_WIDTH, self.LINE_HEIGHT))

        # top_left =     (60,                    0)
        # top_right =    (self.LINE_WIDTH - 55,  0)
        # bottom_left =  (60,                     int(self.LINE_HEIGHT / 2.5) - 1)
        # bottom_right = (self.LINE_WIDTH - 55,      int(self.LINE_HEIGHT / 2.35) - 1)
        # light_points = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32)

        # top_left =     (60,                    int(self.LINE_HEIGHT / 2.5))
        # top_right =    (self.LINE_WIDTH - 55,  int(self.LINE_HEIGHT / 2.35))
        # bottom_left =  (40,                    self.LINE_HEIGHT - 30)
        # bottom_right = (self.LINE_WIDTH-50,    self.LINE_HEIGHT - 25)
        # lightest_points = np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.float32)

        # if self.X11:
        #     cv2.polylines(transformed_image, [np.int32(lightest_points)], isClosed=True, color=(0, 255, 0), thickness=2)
        #     cv2.polylines(transformed_image, [np.int32(light_points)], isClosed=True, color=(0, 255, 0), thickness=2)

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