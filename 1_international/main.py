from core.shared_imports import GPIO, time
start_time = time.perf_counter()

from core.listener import ModeListener
from core.utilities import debug, DisplayManager

import behaviours.line_follow as line_follow
import behaviours.evacuation_zone as evacuation_zone
from hardware.robot import *

listener = ModeListener()
display_manager = DisplayManager()

def main() -> None:
    motors.run(0, 0)
    led.off()
    
    try:
        listener.run()
        display_manager.start()
        
        debug( ["INITIALISATION", f"{time.perf_counter() - start_time:.2f}"], [24, 50] )
        
        while not listener.has_exited():
            if listener.mode.value == 0:
                motors.run(0, 0)
                led.off()
            
            elif listener.mode.value == 1:
                line_follow.main()
                
            elif listener.mode.value == 2:
                evacuation_zone.main()
                
    finally:
        GPIO.cleanup()

if __name__ == "__main__": main()