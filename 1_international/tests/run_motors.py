import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from hardware.robot import *

try:
    while True:
        v1, v2 = list(map(int, input("Enter [v1 v2]: ").split(" ")))
        
        print(v1, v2)
        motors.run(v1, v2)
        
except:
    pass

finally:
    motors.run(0, 0)