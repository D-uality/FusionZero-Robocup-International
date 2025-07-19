from core.shared_imports import cv2, np, time, subprocess
from core.utilities import debug, user_at_host

class EvacuationCamera():
    def __init__(self):        
        self.width  = 320
        self.height = 240
        
        self.device = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
        self.camera = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        
        if not self.camera.isOpened():
            print("Failed to open camera!")
            exit(1)
        
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._set_camera_controls(exposure_value=100)
        
        debug(["INITIALISATION", "E_CAMERA", "✓"], [25, 25, 50])
    
    def _set_camera_controls(self, exposure_value: int, contrast_value: int = 32, sharpness_value: int = 6):
        subprocess.run([
            "v4l2-ctl", "-d", self.device,
            "-c", "auto_exposure=1",
            "-c", f"exposure_time_absolute={exposure_value}"
        ], check=True)
        
        subprocess.run([
            "v4l2-ctl", "-d", self.device,
            f"--set-ctrl=contrast={contrast_value}"
        ], check=True)
        
        subprocess.run([
            "v4l2-ctl", "-d", self.device,
            f"--set-ctrl=sharpness={sharpness_value}"
        ], check=True)


    def capture(self) -> np.ndarray:
        CROP_PRECENTAGE = 0.43
        image = np.zeros((240, 640, 3), dtype=np.uint8)
        
        try:
            while True:
                ok, image = self.camera.read()
                if ok: break
                
                print("CAMERA NOT READY!")

            if user_at_host == "frederick@raspberrypi":
                image = image[:int((1 - CROP_PRECENTAGE) * image.shape[0]), :]
                image = cv2.flip(image, 0)
                image = cv2.flip(image, 1)
            else:
                image = image[int(CROP_PRECENTAGE * image.shape[0]):, :]
                
        except Exception as e:
            debug( [f"ERROR", f"CAMERA", f"{e}"], [30, 20, 50] )
            time.sleep(0.1)
        
        return image
    
    def release(self) -> None:
        self.camera.release()