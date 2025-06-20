import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *

from behaviours.evacuation_zone import align, grab

align_success = align(10, 0.1)
# grab()