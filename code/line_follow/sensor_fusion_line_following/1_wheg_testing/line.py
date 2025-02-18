import time
import colour
import laser_sensors
import touch_sensors
import motors
import evacuation_zone
import config
from main import listener

integral, derivative, last_error = 0, 0, 0
main_loop_count = 0

def follow_line() -> None:
    while True:
        global main_loop_count    
        if listener.get_mode() != 1: break
        
        colour_values = colour.read(display_mapped=True)
        green_signal = green_check(colour_values)
        
        if len(green_signal) != 0: intersection_handling(green_signal)
        else:
            PID(colour_values, 0.7, 0, 0.5)
        
        main_loop_count += 1

def PID(colour_values, kP, kI, kD) -> None:
    global main_loop_count, integral, derivative, last_error
    outer_error = config.outer_multi * (colour_values[0] - colour_values[4])
    inner_error = config.inner_multi * (colour_values[1] - colour_values[3])

    total_error = outer_error + inner_error
    
    integral += total_error
    derivative = total_error - last_error
    
    turn = total_error * kP + integral * kI + derivative * kD
    v1, v2 = config.line_base_speed + turn, config.line_base_speed - turn
    
    motors.run(v1, v2)
    
    config.update_log([f"Line Following ({main_loop_count:.4f})", f"Errors: {outer_error:.2f} {inner_error:.2f} {total_error:.2f}", f"{turn:.2f} -> {v1:.2f} {v2:.2f}"], [25, 29, 29])
    
    last_error = total_error

def green_check(colour_values) -> str:
    signal = ""
    if   colour_values[0] < 30 and colour_values[3] < 50 and colour_values[5] < 20: signal = "left"
    elif colour_values[4] < 30 and colour_values[1] < 50 and colour_values[6] < 20: signal = "right"
    elif colour_values[0] < 30 and colour_values[1] < 30 and colour_values[3] < 30 and colour_values[4] < 30 and colour_values[5] < 20 and colour_values[6] < 20: signal = "double"
    
    return signal

def intersection_handling(signal: str) -> None:
    if signal == "left":
        pass
    elif signal == "right":
        pass
    else:
        pass

