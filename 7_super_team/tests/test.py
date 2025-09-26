import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *

from behaviours.optimized_evacuation import dump

dump("live")