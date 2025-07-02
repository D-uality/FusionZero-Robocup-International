from core.shared_imports import GPIO, time

from core.listener import ModeListener
from core.utilities import *

import behaviours.line_follow as line_follow
import behaviours.evacuation_zone as evacuation_zone
from hardware.robot import *

record = True

listener = ModeListener()
start_display()

def main() -> None:
    motors.run(0, 0)
    led.off()
    start_time = time.perf_counter()
    
    try:
        listener.start()
        
        while not listener.exit_event.is_set():
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
        motors.run(0, 0)
        print("Motors Stopped")
        GPIO.cleanup()
        print("LED's Off")
        stop_display()
        print("Display Stopped")
        evac_camera.release()
        print("Evac Camera Stopped")
        
        claw.lift(claw.pca.servo[claw.lifter_pin].angle)
        claw.close(claw.pca.servo[claw.closer_pin].angle)
        print("Claw Stopped")
        
        time.sleep(0.1)
        if record: save_vfr_video(get_saved_frames())

if __name__ == "__main__": main()