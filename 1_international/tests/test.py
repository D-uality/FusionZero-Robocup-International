import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *
from core.shared_imports import time

from behaviours.evacuation_zone import Search, dump
start_display()

def evac_image():
    image = evac_camera.capture_image()
    green_x = search.triangle(image, "green")
    red_x = search.triangle(image, "red")
    
    live_x = search.classic_live(image, None)
    dead_x = search.ai_dead(image, None)
    
    print(green_x, red_x, live_x, dead_x)
    
    show(image, "image")

def test_dump():
    dump("green")

try:
    search = Search()
    
    input()
    dump("green")

except KeyboardInterrupt:
    pass

finally:
    motors.run(0, 0)