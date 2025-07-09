import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from hardware.robot import *
from core.utilities import *

oled_display.reset()
while True:
    print("displaying hi")
    oled_display.text("hi", 0, 0)