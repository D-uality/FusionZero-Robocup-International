import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))
if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import config
import motors
import colour
import class_modules.gyroscope as gyroscope

def run_input() -> None:
    values = input("[v1, v2]: ").split()
    while True:        
        if len(values) == 0: break
        
        v1, v2 = map(int, values)
        motors.run(v1, v2)

        colour_values = colour.read()
        config.update_log(["TESTING", ", ".join(list(map(str, colour_values)))], [24, 30])
        print()
        
def motor_test() -> None:
    while True:
        motors.run_test(60, 60, 0.5)
        motors.run_test(-60, -60, 0.5)

def gyro_test() -> None:
    gyroscope.initialise()
    
    while True:
        pitch, roll, yaw = gyroscope.read()
        
        if pitch is not None:
            print(f"{pitch:.2f} {roll:.2f} {yaw:.2f}")

if __name__ == "__main__": gyro_test()