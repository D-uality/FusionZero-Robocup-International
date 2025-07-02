import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *
from core.shared_imports import time

from behaviours.evacuation_zone import Search, dump
start_display()

last_live_x, last_dead_x = None, None

def evac_image():
    global last_live_x, last_dead_x
    t0 = time.perf_counter()
    image = evac_camera.capture()
    display_image = image.copy()
        
    green_x = search.triangle(image, display_image, "green")
    red_x = search.triangle(image, display_image, "red")
    
    # live_x = search.classic_live(image, display_image, last_live_x)
    # dead_x = search.hough_dead  (image, display_image, last_dead_x)
    
    # print(green_x, red_x, live_x, dead_x)
    print(green_x, red_x)
    # print(live_x, dead_x)
    
    show(        image, name="image", display=True)
    show(display_image, name="display", display=True)
    
    # last_live_x = live_x
    # last_dead_x = dead_x
    
    print(f"{1/(time.perf_counter() - t0):.2f}")

try:
    search = Search()
    
    while True:        
        evac_image()

except KeyboardInterrupt:
    pass

finally:
    motors.run(0, 0)