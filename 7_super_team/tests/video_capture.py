import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from hardware.robot import *
from core.utilities import *

start_display()

try:
    while True:
        image = evac_camera.capture()
        show(image, "evac")

except KeyboardInterrupt:
    print("SAVING VIDEO")

save_vfr_video(get_saved_frames())