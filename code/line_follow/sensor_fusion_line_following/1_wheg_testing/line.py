import time
from main import listener
from typing import Optional

import config
import colour
import motors
import camera
import cv2
import evacuation_zone

integral, derivative, last_error = 0, 0, 0
main_loop_count = 0
silver_loop_count = 0
min_green_loop_count = 50
running_error = [0 for _ in range(0, 25)]

def follow_line() -> None:
    global main_loop_count, silver_loop_count  
    while True:
        if listener.get_mode() != 1: break

        if main_loop_count % 10 == 0:
            image = camera.capture_array()
            image = camera.perspective_transform(image)

            if config.X11: cv2.imshow("Line Follow", image)
        
        colour_values = colour.read()
        green_signal = green_check(colour_values)
        silver_check(colour_values)

        if len(green_signal) != 0 and main_loop_count > min_green_loop_count: 
            intersection_handling(green_signal, colour_values)
        elif silver_loop_count >= 20:
            silver_loop_count = 0
            evacuation_zone.main()
        else:
            PID(colour_values, 0.9, 0.006, 0.5)

        main_loop_count = main_loop_count + 1 if main_loop_count < 2**31 - 1 else 0
        
        print()

def PID(colour_values: list[int], kP: float, kI: float, kD: float) -> None:
    global main_loop_count, last_green_loop, running_error
    global integral, derivative, last_error
    outer_error = config.outer_multi * (colour_values[0] - colour_values[4])
    inner_error = config.inner_multi * (colour_values[1] - colour_values[3])
 
    total_error = outer_error + inner_error
    running_error[main_loop_count % 25] = total_error

    if abs(total_error) <= 5 and colour_values[1] >= 30 and colour_values[3] >= 30: integral = 0
    elif integral <= -750: integral = -750
    elif integral >=  750: integral =  750
    else: integral += total_error
    derivative = total_error - last_error

    turn = total_error * kP + integral * kI + derivative * kD
    v1, v2 = config.line_base_speed + turn, config.line_base_speed - turn

    motors.run(v1, v2)
    last_error = total_error

    config.update_log([f"PID", f"{main_loop_count}", ", ".join(list(map(str, colour_values))), f"{total_error:.2f} {integral:.2f} {derivative:.2f}", f"{v1:.2f} {v2:.2f}"], [24, 8, 30, 16, 10])

def green_check(colour_values: list[int]) -> str:
    global integral, main_loop_count
    signal = ""    

    if colour_values[0] <= 20 and colour_values[1] <= 20 and colour_values[3] <= 20 and colour_values[4] <= 20 and colour_values[2] <= 20:
        if (colour_values[5] >= 30 and colour_values[6] >= 30):
            main_loop_count = 0
            pass
        else:
            if colour_values[5] <= 50 and colour_values[6] <= 50: signal = "+ double"
            elif colour_values[5] <= 50: signal = "+ left"
            elif colour_values[6] <= 50: signal = "+ right"
            else: pass

    else:
        outer_values = [colour_values[0], colour_values[4]]
        outer_values.sort()      
        inner_values = [colour_values[1], colour_values[3]]
        inner_values.sort()
  
        outer_left_enable  = True if outer_values[0] == colour_values[0] and outer_values[0] < 20 else False
        outer_right_enable = True if outer_values[0] == colour_values[4] and outer_values[0] < 20 else False
        inner_left_enable  = True if inner_values[0] == colour_values[1] and inner_values[0] < 0 else False
        inner_right_enable = True if inner_values[0] == colour_values[3] and inner_values[0] < 0 else False
        
        if (outer_left_enable or outer_right_enable or inner_left_enable or inner_right_enable) and colour_values[2] >= 40 and colour_values[4] <= 30 and colour_values[0] <= 30 and abs(integral) <= 700:   
            motors.run(0, 0, 0.3)
            colour_values = colour.read()
   
            difference = (colour_values[0] + colour_values[1]) - (colour_values[3] + colour_values[4])
            if abs(difference) <= 3:
                signal = ""
            else:
                if (colour_values[5] <= 5 or colour_values[6] <= 5) and abs(colour_values[5] - colour_values[6]) >= 70:
                    signal = "|-" if colour_values[6] < colour_values[5] else "-|"
                else:
                    signal = "|-" if (colour_values[0] + colour_values[1]) >= (colour_values[3] + colour_values[4]) else "-|"
                real_fake = " fake" if colour_values[5] + colour_values[6] >= 120 else " real"
    
                signal += real_fake

    if len(signal) != 0:
        config.update_log(["GREEN CHECK", ", ".join(list(map(str, colour_values)))], [24, 30])
        print()
    
    return signal
        
def intersection_handling(signal: str, colour_values) -> None:
    global main_loop_count
    main_loop_count = 0
    
    config.update_log([f"INTERSECTION HANDLING", f"{signal}"], [24, 16])
    print()
    
    if signal == "+ left":
        motors.run      ( config.line_base_speed,  config.line_base_speed, 0.5)
        motors.run      (-config.line_base_speed,  config.line_base_speed, 1.9)
        motors.run_until(-config.line_base_speed,  config.line_base_speed, colour.read, 3, ">=", 50, "[R] ALIGNING >= 50")
        motors.run_until(-config.line_base_speed,  config.line_base_speed, colour.read, 3, "<=", 45, "[R] ALIGNING <= 45")
        motors.run      ( config.line_base_speed, -config.line_base_speed, 0.35)
  
    if signal == "+ double":
        motors.run      (-config.line_base_speed,  config.line_base_speed, 4)
        motors.run_until(-config.line_base_speed,  config.line_base_speed, colour.read, 3, "<=", 50, "[R] ALIGNING <= 50")
    
    if signal == "+ right":
        motors.run      ( config.line_base_speed,  config.line_base_speed, 0.5)
        motors.run      ( config.line_base_speed, -config.line_base_speed, 1.9)
        motors.run_until( config.line_base_speed, -config.line_base_speed, colour.read, 1, ">=", 50, "[L] ALIGNING >= 50")
        motors.run_until( config.line_base_speed, -config.line_base_speed, colour.read, 1, "<=", 45, "[L] ALIGNING <= 45")
        motors.run      (-config.line_base_speed,  config.line_base_speed, 0.35)
    
    elif signal == "-| real":
        motors.run      ( config.line_base_speed,  config.line_base_speed, 0.5)
        motors.run      (-config.line_base_speed,  config.line_base_speed, 0.8)
        motors.run_until(-config.line_base_speed,  config.line_base_speed, colour.read, 3, "<=", 45, "[R] ALIGNING <= 30")
        motors.run_until(-config.line_base_speed,  config.line_base_speed, colour.read, 3, ">=", 50, "[R] ALIGNING >= 50")
        motors.run      ( config.line_base_speed, -config.line_base_speed, 0.35)
  
    elif signal == "-| fake":
        motors.run_until( config.line_base_speed, -config.line_base_speed, colour.read, 3,  ">=", 50, "[R] ALIGNING >= 50")
        motors.run      ( config.line_base_speed, -config.line_base_speed, 0.3)
        motors.run      ( config.line_base_speed,  config.line_base_speed, 0.5)
 
    elif signal == "|- real":
        motors.run      ( config.line_base_speed,  config.line_base_speed, 0.5)
        motors.run      ( config.line_base_speed, -config.line_base_speed, 0.8)
        motors.run_until( config.line_base_speed, -config.line_base_speed, colour.read, 1, ">=", 50, "[L] ALIGNING >= 50")
        motors.run_until( config.line_base_speed, -config.line_base_speed, colour.read, 1, "<=", 45, "[L] ALIGNING <= 30")
        motors.run      (-config.line_base_speed,  config.line_base_speed, 0.35)
  
    elif signal == "|- fake":
        motors.run_until(-config.line_base_speed,  config.line_base_speed, colour.read, 1,  ">=", 50, "[L] ALIGNING >= 50")
        motors.run      (-config.line_base_speed,  config.line_base_speed, 0.3)
        motors.run      ( config.line_base_speed,  config.line_base_speed, 0.5)
    
    else:
        print(signal)

def silver_check(colour_values: list[int]) -> bool:
    global silver_loop_count
    
    silver_colour_check = [1 if value >= 120 else 0 for value in colour_values[0:5]]
    if sum(silver_colour_check) > 0: silver_loop_count += 1
        
     

# def green_check(colour_values: list[int]) -> str:
#     signal = ""
    
#     error = (colour_values[0] + colour_values[1]) - (colour_values[3] + colour_values[4])
    
#     T_left_enable  = True if error < 0 else False
#     T_right_enable = not T_left_enable
    
#     if colour_values[2] <= 50 or abs(error) <= 10: T_left_enable = T_right_enable = False

#     if colour_values[0] <= 20 and colour_values[1] <= 20 and colour_values[3] <= 20 and colour_values[4] <= 20:        
#         if   colour_values[5] <= 23 and colour_values[6] <= 23: signal += "+ double"
#         elif colour_values[5] <= 23 and colour_values[6] >= 65: signal += "+ left"
#         elif colour_values[6] <= 23 and colour_values[5] >= 65: signal += "+ right"
#         elif colour_values[5] >= 75 and colour_values[6] >= 75 and colour_values[2] <= 10: signal += "+ straight"
#         else:                                                   pass
    
#     if colour_values[0] <= 30 and colour_values[1] <= 20 and T_left_enable:
#         if colour_values[5] <= 35 and colour_values[3] <= 20 and colour_values[6] >= colour_values[5] + 10:
#             signal = "T left"
#         elif colour_values[4] <= 20 and colour_values[5] >= 50 and colour_values[6] >= 50:
#             signal = "fake T left"
#         else: pass
        
#     elif colour_values[4] <= 30 and colour_values[3] <= 20 and T_right_enable:
#         if colour_values[6] <= 35 and colour_values[1] <= 20 and colour_values[5] >= colour_values[6] + 10:
#             signal = "T right"
#         elif colour_values[0] <= 20 and colour_values[5] >= 50 and colour_values[6] >= 50:
#             signal = "fake T right"
#         else: pass
        
#     if len(signal) != 0:
#         config.update_log(["GREEN CHECK", ", ".join(list(map(str, colour_values)))], [24, 30])
#         print()
#     return signal

# def intersection_handling(signal: str, colour_values) -> None:
#     global main_loop_count
#     main_loop_count = 0

#     motors.run(0, 0, 0.2)

#     config.update_log([f"INTERSECTION HANDLING", f"{signal}"], [24, 16])
#     print()
    
#     if signal == "fake T left":
#         motors.run(config.line_base_speed * 1.3, -config.line_base_speed * 0.7, 1.5)
#         motors.run(config.line_base_speed, config.line_base_speed, 0 20)

#     elif signal == "fake T right":
#         motors.run(-config.line_base_speed * 0.7, config.line_base_speed * 1.3, 1.5)
#         motors.run(config.line_base_speed, config.line_base_speed, 0 20)

#     elif signal == "T left":
#         motors.run(-config.line_base_speed, config.line_base_speed, 0.8)
#         motors.run( config.line_base_speed, config.line_base_speed, 0.6)
    
#     elif signal == "T right":
#         motors.run(config.line_base_speed, -config.line_base_speed, 0.8)
#         motors.run(config.line_base_speed,  config.line_base_speed, 0.6)

#     elif signal == "+ left":
#         v1, v2 = -config.line_base_speed * 0.7, config.line_base_speed * 1.3
#         motors.run(v1, v2, 1.3)
#         motors.run_until(v1, v2, colour.read, 3, ">=", 50, "ALIGNING RIGHT")
#         motors.run_until(v1, v2, colour.read, 3, "<=", 45, "ALIGNING RIGHT")
#         motors.run(config.line_base_speed, -config.line_base_speed, 0.23)

#     elif signal == "+ right":
#         v2, v1 = -config.line_base_speed * 0.7, config.line_base_speed * 1.3
#         motors.run(v1, v2, 1.3)
#         motors.run_until(v1, v2, colour.read, 1, ">=", 50, "ALIGNING LEFT")
#         motors.run_until(v1, v2, colour.read, 1, "<=", 45, "ALIGNING LEFT")
#         motors.run(-config.line_base_speed, config.line_base_speed, 0.23)

#     elif signal == "+ straight":
#         motors.run(config.line_base_speed * 1.2, config.line_base_speed * 1.2, 0.35)

#     elif signal == "+ double":
#         motors.run(config.line_base_speed * 2, -config.line_base_speed * 2, 2)
#         motors.run_until(config.line_base_speed * 2, -config.line_base_speed * 2, colour.read, 1, "<=", 50, "SPINNING")
#     else:
#         pass
