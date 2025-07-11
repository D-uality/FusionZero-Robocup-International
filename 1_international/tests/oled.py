import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *
from core.shared_imports import time

oled_display.clear()
oled_display.draw_circle(64, 32, 40, 7, update_display=True)
F_size = 30
oled_display.text("F", int(64 - F_size / 4) + 1, int(32 - F_size / 2) - 3, size=F_size, font_family="cambria", update_display=True)

# time.sleep(1)
# oled_display.draw_spiral_cutouts(64, 32, 39, 49, 5, update_display=True)
# time.sleep(1)


# time.sleep(1)