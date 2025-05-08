import time
import numpy as np
import cv2
from typing import Optional
from robot import motors, laser_sensors

HEIGHT = 240
WIDTH = 320
        
class ClassicSearch():
    def __init__(self):
        self.__display = True
        self.width = WIDTH
        self.height = HEIGHT
    
    def find(self, image: np.ndarray, vicimt_count: int):
        if vicimt_count <= 2:
            result = self.live(image)
        else:
            result = self.dead(image)
        
        return result
    
    def live(self, image: np.ndarray) -> Optional[int]:
        spectral_threshold = 200

        kernal_size = 7
        spectral_highlights = cv2.inRange(image, (spectral_threshold, spectral_threshold, spectral_threshold), (255, 255, 255))
        spectral_highlights = cv2.dilate(spectral_highlights, np.ones((kernal_size, kernal_size), np.uint8), iterations=1)

        contours, _ = cv2.findContours(spectral_highlights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours: return None
        if self.__display: cv2.drawContours(image, contours, -1, (0, 0, 255), 1)

        valid_contours = []
        
        for contour in contours:
            _, y, w, h = cv2.boundingRect(contour)
            # print(cv2.contourArea(contour))
            
            # look for contours in the top quarter
            if (y + h/2 < 50
                and cv2.contourArea(contour) < 700):
                valid_contours.append(contour)
        
        if len(valid_contours) == 0: return None

        largest_contour = max(valid_contours, key=cv2.contourArea)
        x, _, w, _ = cv2.boundingRect(largest_contour)
        
        if self.__display: cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
        return int(x + w/2)
    
    def dead(self, image: np.ndarray) -> Optional[int]:
        def circularity_check(contour: np.ndarray, threshold: float) -> bool:
            perimeter = cv2.arcLength(contour, True)
            area = cv2.contourArea(contour)

            if perimeter == 0: return False

            circularity = (4 * np.pi * area) / (perimeter ** 2)

            if circularity > threshold: return True
            return False
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        black_mask = cv2.inRange(hsv_image, (0, 0, 0), (179, 255, 30))
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # If there are no contours, or there are many, return None
        if not contours or len(contours) > 5: return None
        
        if self.__display: cv2.drawContours(image, contours, -1, (0, 0, 255), 1)
        
        valid_contours = []
        
        for contour in contours:
            _, y, _, h = cv2.boundingRect(contour)
            if (y + h/2 < self.height / 2 + 30
                and circularity_check(contour, 0.5)
                and cv2.contourArea(contour) > 100
                and cv2.contourArea(contour) < 10000):
                valid_contours.append(contour)
        
        if len(valid_contours) == 0: return None
        
        largest_contour = max(valid_contours, key=cv2.contourArea)

        x, _, w, _ = cv2.boundingRect(largest_contour)
        
        if self.__display: cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)

        return int(x + w/2)

class EvacuationCamera():
    def __init__(self):
        try:
            self.width = WIDTH
            self.height = HEIGHT
            self.camera = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
            
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.height)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.width)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        except Exception as error:
            print(f"Failed to open camera! {error}")
            exit()

    def capture_image(self) -> np.ndarray:
        while True:
            ok, image = self.camera.read()
            if ok: break
        
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE) 
        image = image[130:, :]
        
        return image
    
    def release(self) -> None:
        self.camera.release()
        
class EvacuationZone():
    def __init__(self, camera: EvacuationCamera, victim_count: int = 0):
        self.victim_count = victim_count
        self.start_time = time.perf_counter()
        
        self.classic_search = ClassicSearch()
        self.camera = camera
    
    def search(self):
        while True:
            image = self.camera.capture_image()
            
            self.classic_search.find(image, self.victim_count)
            
            cv2.imshow("image", image)
            cv2.waitKey(1)
            
    def wall_follow(self):
        laser_values = laser_sensors.read()
        
        print(laser_values)
        
    
def main() -> None:
    try:
        camera = EvacuationCamera()
        evacuation_zone = EvacuationZone(camera)
            
        
            
    finally:
        camera.release()
        
if __name__ == "__main__":
    cv2.startWindowThread()
    main()