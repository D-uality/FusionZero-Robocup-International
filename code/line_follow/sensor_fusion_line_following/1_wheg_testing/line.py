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

def follow_line() -> None:
    while True:
        global main_loop_count    
        if listener.get_mode() != 1: break

        if main_loop_count % 10 == 0:
            image = camera.capture_array()
            image = camera.perspective_transform(image)

            if config.X11: cv2.imshow("Line Follow", image)
        
        colour_values = colour.read(display_mapped=True)
        green_signal = green_check(colour_values)
        
        if "fake" in green_signal:
            if "left" in green_signal:
                colour_values[0] += 40
                colour_values[1] += 40
                return colour_values

            elif "right" in green_signal:
                colour_values[3] += 40
                colour_values[4] += 40
                return colour_values
        
        PID(colour_values, 0.7, 0, 1)

        main_loop_count = main_loop_count + 1 if main_loop_count < 2**31 - 1 else 0

def PID(colour_values, kP, kI, kD) -> None:
    global main_loop_count, integral, derivative, last_error
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
    
    config.update_log([f"Line Following ({main_loop_count:.4g})", f"Errors: {outer_error:.2f} {inner_error:.2f} {total_error:.2f}", f"{integral:.2f} {derivative:.2f}",f"{turn=:.2f} -> {v1:.2f} {v2:.2f}"], [25, 29, 13, 29])
    
    last_error = total_error

def green_check(colour_values) -> str:
    signal = ""
    
    if colour_values[0] < 30 and colour_values[1] < 20:
        if colour_values[5] < 5: signal = "left"
        else:                    signal = "fake left"
    if colour_values[3] < 20 and colour_values[4] < 30:
        if colour_values[5] < 5: signal = "right"
        else:                    signal = "fake right"
    
    return signal

def intersection_handling(signal: str) -> None:
    if "fake" in signal: return None

    motors.run(0, 0)
    input()
    if signal == "left":
        pass
    elif signal == "right":
        pass
    else:
        pass

