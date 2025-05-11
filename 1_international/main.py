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
    motors.claw(0)
    led.off()
    
    try:
        listener.run()
        display_manager.start()
        
        debug( ["INITIALISATION", f"{time.perf_counter() - start_time}"], [24, 50] )
        
        while listener.has_exited() == False:
            if listener.mode == 9:
                listener.stop()
            
            elif listener.mode == 0:
                motors.run(0, 0)
                led.on()

            elif mode == 1:
                line.main()

            elif mode == 2:
                gyro_values = gyroscope.read()
                gyro_values = gyro_values if gyro_values is not None else ""
                # if is not None:
                debug(["MODE 2", f"Touch: {touch_sensors.read()}   Lasers: {laser_sensors.read()}   Colour: {colour_sensors.read()}   Gyro: {gyro_values} "], [30, 50])

            elif mode == 9: listener.exit_event.set()

    finally:
        GPIO.cleanup()

if __name__ == "main": main()