import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import oled_display
import laser_sensors
import touch_sensors
import motors
import camera
import gyroscope
from colour_sensors import cCOLOUR_SENSORS

def debug(data: list[str], coloumn_widths: list[int], separator: str = "|"):
    formatted_cells = [f"{cell:^{width}}" for cell, width in zip(data, coloumn_widths)]
    print(f" {separator} ".join(formatted_cells))
    
def main() -> None:
    start_time = time.perf_counter()
    
    oled_display.initialise()
    laser_sensors.initialise()
    touch_sensors.initialise()
    motors.initialise()
    camera.initialise("line")
    gyroscope.initialise()
    colour_sensors = cCOLOUR_SENSORS()
    
    debug(["INITIALISATION", f"{time.perf_counter() - start_time:.2f}"], [24, 24])

    while True:
        pass
    
if __name__ == "__main__":
    main()