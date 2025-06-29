from core.shared_imports import cv2, np, time, socket, getpass
from core.utilities import debug

class EvacuationCamera():
    def __init__(self):
        username = getpass.getuser()
        hostname = socket.gethostname()
        self.user_at_host = f"{username}@{hostname}"
        
        self.width  = 320 * 2
        self.height = 240 * 2
        
        device = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
        self.camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
        
        if not self.camera.isOpened():
            print("Failed to open camera!")
            exit(1)
        
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        debug(["INITIALISATION", "E_CAMERA", "✓"], [25, 25, 50])

    def capture(self, evac = True) -> np.ndarray:
        image = np.zeros((240, 640, 3), dtype=np.uint8)
        
        try:
            while True:
                ok, image = self.camera.read()
                if ok: break
                
                print("CAMERA NOT READY!")
            
            if evac: image = image[:int(0.48 * self.height), :]
            else: image = image[int(0.8 * self.height):, :]

            if self.user_at_host == "frederick@raspberrypi":
                image = cv2.flip(image, 0)
                image = cv2.flip(image, 1)
        
        except Exception as e:
            debug( [f"ERROR", f"CAMERA", f"{e}"], [30, 20, 50] )
            time.sleep(0.1)
        
        return image
    
    def release(self) -> None:
        self.camera.release()