import os
import sys
import threading

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

def listener():
    valid_modes = {'0', '1', '2', '3', '4', '5', '6'}

    while True:
        print("[0] Nothing")
        print("[1] Line Follow only")
        print("[2] Evacuation only")
        print("[3] Combined")
        print("[4] Read Sensors")
        print("[5] Calibration")
        print("[6] Testing")
        print("[9] Exit program")
        
        mode = input()
        if mode in valid_modes: mode = int(mode)
        
        print("Enter a valid mode!")

listener_thread = threading.Thread(target=listener, daemon=True)
listener_thread.start()

def main():
    oled_display.initialise()
    laser_sensors.initialise()
    touch_sensors.initialise()
    motors.initialise()
    camera.initialise()
    
    motors.run(0, 0)
    oled_display.reset()
    
    try:
        while True:
            if   mode == 1: line.follow_line()
            
            elif mode == 5: colour.calibration(auto_calibrate=True)
            elif mode == 6: testing.run_input()
            
            elif mode == 9: exit()
    
    except KeyboardInterrupt:
        print("Exiting gracefully... ")
        
    finally:
        camera.close()
        oled_display.reset()
        motors.run(0, 0)
    
if __name__ == "__main__": main()