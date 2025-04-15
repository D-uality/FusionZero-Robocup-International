import os
import sys
import time
from listener import listener

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import led
from motors import cMOTORS
from camera import cCAMERA

def debug(data: list[str], coloumn_widths: list[int], separator: str = "|"):
    formatted_cells = [f"{cell:^{width}}" for cell, width in zip(data, coloumn_widths)]
    print(f" {separator} ".join(formatted_cells))
    
def main() -> None:
    start_time = time.perf_counter()

    motors = cMOTORS()
    motors.initialise()
    camera = cCAMERA("line")
    camera.initialise("line")
    
    debug(["INITIALISATION", f"{time.perf_counter() - start_time:.2f}"], [24, 24])
    
    try:
        while not listener.has_exited():
            mode = listener.get_mode()
            
            if mode == 0:
                motors.run(0, 0)
                led.off()

            elif mode == 1:
                line.main()
            
            elif mode == 9: listener.exit_event.set()

    finally:
        camera.close()
        led.off()
        motors.run(0, 0)
    
if __name__ == "__main__":
    main()