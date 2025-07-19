import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from hardware.robot import *
from core.utilities import *

start_display()

def camera_claw():
    image = evac_camera.capture()
    show(image, "image")
    mode, angle = list(map(int, input("Enter [Mode Angle]: ").split(" ")))
    
    if mode == 0:
        claw.lift(angle)
        
    else:
        claw.close(angle)

def set_claw():
    mode, angle = list(map(int, input("Enter [Mode Angle]: ").split(" ")))
    
    if mode == 0:
        claw.lift(angle)
        
    else:
        claw.close(angle)


def read_claw():
    claw.read()
    print(claw.spaces)

while True:
    read_claw()
    # set_claw()
