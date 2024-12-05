from picamera2 import Picamera2
from libcamera import Transform

# Config variables
WIDTH, HEIGHT = 320, 240
FLIP = False
X11 = True

# Initialize Picamera2
camera = Picamera2()

# Create a correct transform object from libcamera.Transform
camera_config = camera.create_preview_configuration(main={"format": "RGB888", "size": (WIDTH, HEIGHT)}, transform=Transform(vflip=FLIP, hflip=FLIP))

# Configure and start the camera
camera.configure(camera_config)
camera.start()