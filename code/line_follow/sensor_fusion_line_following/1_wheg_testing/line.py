import time
import colour
import laser_sensors
import touch_sensors
import motors
import evacuation_zone
import config


integral, derivative, last_error = 0, 0, 0
main_loop_count = 0

def follow_line():
    while True:
        global main_loop_count    
        
        colour_values = colour.read()
        PID(colour_values, 0.7, 0, 0.5)
        
        main_loop_count += 1
    
def PID(colour_values, kP, kI, kD):
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