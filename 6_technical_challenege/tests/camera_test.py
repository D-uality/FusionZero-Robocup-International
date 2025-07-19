import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from hardware.robot import *
from core.utilities import *

start_display()

while True:
    line = camera.capture_array()
    evac = evac_camera.capture()
    
    show(line, "line")
    show(evac, "evac")