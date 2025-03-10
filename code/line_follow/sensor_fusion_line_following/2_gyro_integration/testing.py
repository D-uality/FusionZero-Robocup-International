import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))
if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import config
import motors
import colour
import gyroscope
import camera
import numpy as np
import cv2
import led

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
        motors.run(18, 18)

def gyro_test() -> None:
    gyroscope.initialise()
    
    while True:
        pitch, roll, yaw = gyroscope.read()
        
        if pitch is not None:
            print(f"{pitch:.2f} {roll:.2f} {yaw:.2f}")

def camera_line_follow():
    try:
        kP = 0.5
        last_angle = 0
        camera.initialise(config.LINE_WIDTH, config.LINE_HEIGHT)
        led.on()
        
        while True:
            image = camera.capture_array()
            transformed_image = camera.perspective_transform(image)
            line_image = transformed_image.copy()
            
            colour_values = colour.read()
            
            black_contour, black_image = camera.find_line_black_mask(transformed_image, line_image, last_angle)
            angle = camera.calculate_angle(black_contour, line_image, last_angle)
            
            error = 90 - angle
            turn = error * kP
            motors.run(30 - turn, 30 + turn)
            
            last_angle = angle
            if config.X11: cv2.imshow("Line", line_image)
            
            config.update_log(["GAP HANDLING", f"{error} {angle}"], [24, 18])
            print()
            
    finally:
        camera.close()
        motors.run(0, 0)
        led.off()

if __name__ == "__main__": motor_test()
