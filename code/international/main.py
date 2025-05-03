import os
import sys
import time
from listener import C_MODE_LISTENER

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import led
from robot import *
from utils import debug
import line

def main() -> None:
    start_time = time.perf_counter()

    listener = C_MODE_LISTENER()
    listener.start()
    
    debug(["INITIALISATION", f"{time.perf_counter() - start_time:.2f}"], [24, 24])
    
    try:
        while not listener.has_exited():
            mode = listener.get_mode()
            
            if mode == 0:
                motors.run(0, 0)
                led.on()

            elif mode == 1:
                line.main()

            elif mode == 2:
                # debug(["MODE 2", f"Touch: {touch_sensors.read()}   Lasers: {laser_sensors.read()}   Colour: {colour_sensors.read()}"], [30, 50])
                gyro_values = gyroscope.read()
                if gyro_values is not None:
                    print(gyro_values)

            elif mode == 9: listener.exit_event.set()

    finally:
        camera.close()
        led.off()
        motors.run(0, 0)
        led.close()
    
if __name__ == "__main__":
    main()