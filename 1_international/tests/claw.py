import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from hardware.robot import *
from core.utilities import *

start_display()

try:
    while True:
        image = evac_camera.capture_image()
        show(image, "image")
        mode, angle = list(map(int, input("Enter [Mode Angle]: ").split(" ")))
        
        if mode == 0:
            claw.lift(angle)
            
        else:
            claw.close(angle)
        
except:
    pass

finally:
    motors.run(0, 0)