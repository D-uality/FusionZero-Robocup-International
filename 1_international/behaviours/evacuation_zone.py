from core.shared_imports import cv2
from hardware.robot import *
from core.utilities import *

start_display()

def main() -> None:
    # while True:
    #     print("evac")
    #     image = evac_camera.capture_image()
        
    #     show(image, name="image")
    
    input("Waiting to begin sequence")
    
    claw.close(0)
    
    input("")
    
    claw.lift(20, 0.02)
    
    input("")
    
    claw.close(90)
    
    input("")
    
    claw.lift(150, 0.02)