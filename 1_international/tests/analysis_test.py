import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *
from core.shared_imports import time

# from behaviours.evacuation_zone import Search, dump
from behaviours.optimized_evacuation import op_search
# from behaviours.evacuation_zone import search
start_display()

last_live_x, last_dead_x = None, None

def evac_image():
    global last_live_x, last_dead_x
    t0 = time.perf_counter()
    image = evac_camera.capture()
    display_image = image.copy()
        
    green_x = op_search.triangle(image, display_image, "green")
    red_x = op_search.triangle(image, display_image, "red")
    
    # live_x = op_search.live(image, display_image, last_live_x)
    # dead_x = op_search.dead(image, display_image, last_dead_x)
    # dead_x = search.hough_dead(image, display_image, last_dead_x)
    
    # print(green_x, red_x, live_x, dead_x)
    print(green_x, red_x)
    # print(live_x, dead_x)
    
    # show(        image, "image")
    show(display_image, "display")
    
    # last_live_x = live_x
    # last_dead_x = dead_x
    
    # print(f"{1/(time.perf_counter() - t0):.2f}")

while True:        
    evac_image()
    print()