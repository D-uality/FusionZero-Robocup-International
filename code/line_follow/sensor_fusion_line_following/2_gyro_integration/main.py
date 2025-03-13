import os
import sys
from listener import listener

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import camera
import colour
import laser_sensors
import touch_sensors
import oled_display
import motors
import gyroscope
import led

import line
import evacuation_zone
import testing
import config

def main() -> None:
    oled_display.initialise()
    # laser_sensors.initialise()
    touch_sensors.initialise()
    motors.initialise()
    camera.initialise(config.LINE_WIDTH, config.LINE_HEIGHT)
    gyroscope.initialise()
    
    motors.run(0, 0)
    oled_display.reset()
    led.off()
    
    try:
        while not listener.has_exited():
            mode = listener.get_mode()
            
            if mode == 0:
                motors.run(0, 0)

            elif mode == 1:
                line.main()
                
            elif mode == 2:
                evacuation_zone.main()
            
            elif mode == 4:
                config.update_log(["READING SENSORS", ", ".join(list(map(str, colour.read()))), ", ".join(list(map(str, touch_sensors.read())))], [24, 30, 10])
                print()

            elif mode == 5:
                colour.calibration(auto_calibrate=False)
                colour.load_calibration_values()
                listener.mode = 0

            elif mode == 6:
                testing.run_input()
            
            elif mode == 9: listener.exit_event.set()

    finally:
        camera.close()
        oled_display.reset()
        motors.run(0, 0)
    
if __name__ == "__main__": main()