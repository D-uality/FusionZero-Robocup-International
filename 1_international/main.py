from core.shared_imports import GPIO, time
start_time = time.perf_counter()

from core.listener import ModeListener
from core.utilities import debug

import behaviours.line_follow as line_follow
import behaviours.evacuation_zone as evacuation_zone
from hardware.robot import *

listener = ModeListener()

def main() -> None:
    motors.run(0, 0)
    led.off()
    start_time = time.perf_counter()
    
    try:
        listener.run()
        # display_manager.start()
        
        debug( ["INITIALISATION", f"{time.perf_counter() - start_time:.2f}"], [24, 50] )
        
        while not listener.has_exited():
            if listener.mode.value != 1:
                start_time = time.perf_counter()

            if listener.mode.value == 0:
                motors.run(0, 0)
                led.off()
                
            # elif listener.mode.value == 1:
                # line_follow.main(start_time)
                
            elif listener.mode.value == 2:
                evacuation_zone.main()
            
            elif listener.mode.value == 3:
                led.on()
                # gyro_values = gyroscope.read()
                # gyro_values = gyro_values if gyro_values is not None else ""
                debug(["MODE 2", f"Touch: {touch_sensors.read()}   Lasers: {laser_sensors.read()}"], [30, 50])

            elif listener.mode.value == 9: listener.exit_event.set()

    except KeyboardInterrupt:
        print("Exiting gracefully!")
    
    finally:
        GPIO.cleanup()
        evac_camera.release()
        
        claw.lift(160)
        claw.close(90)
        
        motors.run(0, 0)

if __name__ == "__main__": main()