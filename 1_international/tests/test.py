import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.shared_imports import math
from core.utilities import *
from hardware.robot import *

try:
    motors.ease(-45, 60, 0, -2, 3)
        
except KeyboardInterrupt:
    pass

finally:
    motors.run(0, 0)
