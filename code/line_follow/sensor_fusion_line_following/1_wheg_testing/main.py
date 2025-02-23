import os
import sys
from listener import C_MODE_LISTENER

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import camera
import colour
import laser_sensors
import touch_sensors
import oled_display

import line
import testing
import motors
import config

listener = C_MODE_LISTENER()
listener.start()

def main() -> None:
    oled_display.initialise()
    # laser_sensors.initialise()
    touch_sensors.initialise()
    motors.initialise()
    camera.initialise()
    
    motors.run(0, 0)
    oled_display.reset()
    
    try:
        while not listener.has_exited():
            mode = listener.get_mode()
            
            if mode == 0:
                motors.run(0, 0)

            elif mode == 1:
                line.follow_line()
            
            elif mode == 4:
                config.update_log(["READING SENSORS", ", ".join(list(map(str, colour.read())))], [24, 30])
                print()

            elif mode == 5:
                colour.calibration(auto_calibrate=True)
                listener.mode = 0

            elif mode == 6:
                testing.run_input()
            
            elif mode == 9: listener.exit_event.set()

    finally:
        camera.close()
        oled_display.reset()
        motors.run(0, 0)
    
if __name__ == "__main__": main()