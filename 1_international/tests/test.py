import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from core.utilities import *
from hardware.robot import *
from core.shared_imports import time

try:
    initial_distance = 0
    while True:
        initial_distance = laser_sensors.read([1])[0]
        if initial_distance is not None: break
        
    print(initial_distance)
    
    while True:
        motors.run(-20,  20)
        distance = laser_sensors.read([1])[0]
        
        if distance > initial_distance + 3: break
        
        debug( ["ALIGNING [CENTRE]", "TURNING TILL NOT", f"{distance}"], [15, 15, 15])

    motors.run(0, 0, 0.3)

    t0 = time.perf_counter()
    motors.run(20, -20, 0.3)
    
    while True:
        motors.run(20, -20)
        distance = laser_sensors.read([1])[0]
        
        if distance > initial_distance + 3: break
        
        debug( ["ALIGNING [CENTRE]", "RECORDING TIME", f"{distance}"], [15, 15, 15])
    
    t1 = time.perf_counter()
    
    print(f"Total time was: {t1 - t0}!")
    
    motors.run(-20, 20, (t1-t0)/2)
    motors.run(0, 0)

except KeyboardInterrupt:
    pass

finally:
    motors.run(0, 0)