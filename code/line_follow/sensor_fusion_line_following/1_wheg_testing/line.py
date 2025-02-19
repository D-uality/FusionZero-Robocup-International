import time
from main import listener
from typing import Optional

import config
import colour
import motors
import camera
import cv2

integral, derivative, last_error = 0, 0, 0
main_loop_count = 0
min_green_loop_count, last_green_loop = 50, 0

def follow_line() -> None:
    while True:
        global main_loop_count    
        if listener.get_mode() != 1: break

        if main_loop_count % 10 == 0:
            image = camera.capture_array()
            image = camera.perspective_transform(image)

            if config.X11: cv2.imshow("Line Follow", image)
        
        colour_values = colour.read()
        green_signal = green_check(colour_values)

        if len(green_signal) != 0 and main_loop_count - last_green_loop > min_green_loop_count: 
            intersection_handling(green_signal, colour_values)
        else:
            PID(colour_values, 0.75, 0, 1.2)

        main_loop_count = main_loop_count + 1 if main_loop_count < 2**31 - 1 else 0
        
        print()

def PID(colour_values: list[int], kP: float, kI: float, kD: float) -> None:
    global main_loop_count, last_green_loop
    global integral, derivative, last_error
    outer_error = config.outer_multi * (colour_values[0] - colour_values[4])
    inner_error = config.inner_multi * (colour_values[1] - colour_values[3])

    total_error = outer_error + inner_error
    
    if abs(total_error) < 10: integral = 0
    elif integral < -750: integral = -750
    elif integral >  750: integral = 750
    else: integral += total_error
    derivative = total_error - last_error
    
    turn = total_error * kP + integral * kI + derivative * kD
    v1, v2 = config.line_base_speed + turn, config.line_base_speed - turn
    
    motors.run(v1, v2)
    last_error = total_error
    
    config.update_log([f"PID", f"{main_loop_count} {last_green_loop}", ", ".join(list(map(str, colour_values))), f"{total_error:.2f} {integral:.2f} {derivative:.2f}", f"{v1:.2f} {v2:.2f}"], [24, 16, 30, 16, 10])
    

def green_check(colour_values: list[int]) -> str:
    global last_green_loop, main_loop_count

    signal = ""
    
    T_left_enable  = True if (colour_values[0] + colour_values[1]) - (colour_values[3] + colour_values[4]) < 0 else False
    T_right_enable = not T_left_enable
    if colour_values[2] <= 30: T_left_enable = T_right_enable = False

    if colour_values[0] <= 10 and colour_values[1] <= 10 and colour_values[3] <= 10 and colour_values[4] <= 10:
        signal += "+ "
        
        if   colour_values[5] <= 15 and colour_values[6] <= 15: signal += "double"
        elif colour_values[5] <= 15 and colour_values[6] >= 65: signal += "left"
        elif colour_values[6] <= 15 and colour_values[5] >= 65: signal += "right"
        elif colour_values[5] >= 75 and colour_values[6] >= 75: signal += "straight"
        else:                                                   singal = ""
    
    if colour_values[0] <= 30 and colour_values[1] <= 20 and T_left_enable:
        if colour_values[5] <= 35 and colour_values[3] <= 20 and colour_values[6] >= colour_values[5] + 10:    signal = "T left"
        elif colour_values[4] <= 10:                                                signal = "fake T left"
        else: pass
        
    elif colour_values[4] <= 30 and colour_values[3] <= 20 and T_right_enable:
        if colour_values[6] <= 35 and colour_values[1] <= 20 and colour_values[5] >= colour_values[6] + 10:    signal = "T right"
        elif colour_values[0] <= 10:                                                signal = "fake T right"
        else: pass
        
    if len(signal) != 0:
        config.update_log(["GREEN CHECK", ", ".join(list(map(str, colour_values)))], [24, 30])
        print()
    return signal

def intersection_handling(signal: str, colour_values) -> None:
    global last_green_loop, main_loop_count
    last_green_loop = main_loop_count

    config.update_log([f"INTERSECTION HANDLING", f"{signal}"], [24, 16])
    print()
    
    if signal == "fake T left":
        motors.run((config.line_base_speed + 10) * 1.6, -(config.line_base_speed + 10) * 0.7, 0.8)
        motors.run(config.line_base_speed, config.line_base_speed, 0.2)

    elif signal == "fake T right":
        motors.run(-(config.line_base_speed + 10) * 0.7, (config.line_base_speed + 10) * 1.6, 0.8)
        motors.run(config.line_base_speed, config.line_base_speed, 0.2)

    elif signal == "T left":
        motors.run(0, 0)
        input()
        pass
    
    elif signal == "T right":
        motors.run(0, 0)
        input()
        pass

    elif signal == "+ left":
        motors.run(0, 0)
        input()
        pass

    elif signal == "+ right":
        motors.run(0, 0)
        input()
        pass

    elif signal == "+ straight":
        motors.run(config.line_base_speed * 1.2, config.line_base_speed * 1.2, 0.35)

    elif signal == "+ double":
        motors.run(0, 0)
        input()
        pass

    else:
        pass
