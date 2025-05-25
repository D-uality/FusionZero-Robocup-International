import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *
from core.shared_imports import time

from behaviours.evacuation_zone import Search
start_display()

try:
    searcher = Search()
    last_x = None
    
    while True:
        image = evac_camera.capture_image()
        last_x, image = searcher.classic_live(image, last_x)
        
        show(image, "image")

except KeyboardInterrupt:
    pass

finally:
    motors.run(0, 0)