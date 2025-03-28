import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import camera
import laser_sensors
import touch_sensors
import oled_display
import motors
import gyroscope
import led
import line
import config

def main() -> None:
    start_time = time.time()
    oled_display.initialise()
    laser_sensors.initialise()
    touch_sensors.initialise()
    motors.initialise()
    camera.initialise("line")
    gyroscope.initialise()
    
    motors.run(0, 0)
    oled_display.reset()
    led.off()
    
    config.update_log(["INITIALISATION", f"({time.time() - start_time:.2f})"], [24, 15])
    print()
    
    try:
        while True:
            line.main()

    finally:
        camera.close()
        oled_display.reset()
        motors.run(0, 0)
    
if __name__ == "__main__": main()