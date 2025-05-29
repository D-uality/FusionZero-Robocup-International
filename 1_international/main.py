from core.shared_imports import GPIO, time

from core.listener import ModeListener
from core.utilities import debug

from hardware.robot import *
import behaviours.line_follow as line_follow
import behaviours.evacuation_zone as evacuation_zone

listener = ModeListener()

def main() -> None:
    motors.run(0, 0)
    led.off()
    
    try:
        listener.run()
        
        while not listener.has_exited():
            if listener.mode.value != 1:
                start_time = time.perf_counter()

            if listener.mode.value == 0:
                motors.run(0, 0)
                led.off()
                
            elif listener.mode.value == 1:
                line_follow.main(start_time)
                
            elif listener.mode.value == 2:
                evacuation_zone.main()
            
            elif listener.mode.value == 3:
                led.on()
                
                gyro_values = gyroscope.read()
                gyro_values = gyro_values if gyro_values is not None else ""
                debug( [
                    "READING", 
                    " ".join(list(map(str, touch_sensors.read()))),
                    " ".join(list(map(str, laser_sensors.read()))), 
                    " ".join(list(map(str, gyro_values)))
                    ], [25, 20, 20, 20]
                )

            elif listener.mode.value == 9: listener.exit_event.set()

    except KeyboardInterrupt:
        print("Exiting gracefully!")
    
    finally:
        GPIO.cleanup()
        evac_camera.release()
        camera.close()
        
        claw.lift(160)
        claw.close(90)
        motors.run(0, 0)

if __name__ == "__main__": main()