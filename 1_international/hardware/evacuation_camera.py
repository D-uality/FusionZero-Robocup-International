from core.shared_imports import cv2, np, time, mp
from core.utilities import debug

class EvacuationCamera():
    def __init__(self):
        self.width  = 320 * 2
        self.height = 240 * 2
        
        device = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
        self.camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
        
        if not self.camera.isOpened():
            raise "Failed to open camera!"
        
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.image_queue       = mp.Queue(maxsize=1)
        self.exit_event        = mp.Event()
        self.__capture_process = mp.Process(target=self.__background_capture)
        
        self.start()
        debug(["INITIALISATION", "E_CAMERA", "✓"], [25, 25, 50])

    def start(self) -> None:
        self.__capture_process.start()
        
    def stop(self) -> None:
        self.exit_event.set()
        
        if self.__capture_process.is_alive():
            self.__capture_process.terminate()
            self.__capture_process.join()            
    
    def release(self) -> None:
        self.stop()
        self.camera.release()
    
    def capture(self) -> np.ndarray:
        if not self.image_queue.empty():
            return self.image_queue.get_nowait()
        
        else:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
    
    def __background_capture(self) -> None:        
        while not self.exit_event.is_set():
            t0 = time.perf_counter()
            
            while True:
                ok, image = self.camera.read()
                if ok: break
                if time.perf_counter() - t0 > 1:
                    print("Failed to capture image from evacuation camera!")
                    return
            
            image = image[:int(0.48 * self.height), :]
            
            image = cv2.flip(image, 0)
            image = cv2.flip(image, 1)
            
            self.image_queue.put(image)