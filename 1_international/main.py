from core.shared_imports import GPIO, time, random

from core.listener import listener
from core.utilities import *

import behaviours.line as line
import behaviours.optimized_evacuation as optimized_evacuation_zone
from hardware.robot import *
from behaviours.robot_state import RobotState
from behaviours.line_follower import LineFollower

record = True
robot_state = RobotState()
line_follow = LineFollower(robot_state)

start_display()

def main() -> None:
    motors.run(0, 0)
    led.off()
    start_time = time.perf_counter()
    random.seed(time.time())
    
    try:
        listener.start()
        oled_display.display_logo()

        while not listener.exit_event.is_set():
            if listener.mode.value != 1:
                start_time = time.perf_counter()

            if listener.mode.value == 0:
                motors.run(0, 0)
                led.off()
                robot_state.reset()
                
            elif listener.mode.value == 1:
                line.main(start_time, robot_state, line_follow)
                
            elif listener.mode.value == 2:
                optimized_evacuation_zone.main()
            
            elif listener.mode.value == 3:
                led.on()
                
                gyro_values = gyroscope.read()
                gyro_values = gyro_values if gyro_values is not None else ""
                debug( [
                    "READING", 
                    " ".join(list(map(str, touch_sensors.read()))),
                    " ".join(list(map(str, colour_sensors.read()))),
                    f"{silver_sensor.read()}",
                    " ".join(list(map(str, laser_sensors.read()))), 
                    " ".join(list(map(str, gyro_values)))
                    ], [25, 20, 20, 20, 20, 20]
                )
            
            elif listener.mode.value == 4:
                colour_sensors.calibrate()
            elif listener.mode.value == 5:
                silver_sensor.calibrate()

            elif listener.mode.value == 9: listener.exit_event.set()

    except KeyboardInterrupt:
        print("Exiting gracefully!")
    
    finally:
        oled_display.clear()
        oled_display.text("EXITING", 0, 0)
        
        motors.run(0, 0)
        print("Motors Stopped")
        led.off()
        GPIO.cleanup()
        print("LED's Off")
        
        stop_display()
        print("Display Stopped")
        camera.close()
        print("Camera Stopped")
        evac_camera.release()
        print("Evac Camera Stopped")

        
        claw.lift(claw.pca.servo[claw.lifter_pin].angle)
        claw.close(claw.pca.servo[claw.closer_pin].angle)
        print("Claw Stopped")
        
        time.sleep(0.1)
        
        if record: 
            print("Creating Video...")
            oled_display.text("VIDEO", 0, 15)
            save_vfr_video(get_saved_frames())

if __name__ == "__main__": main()