import time
from listener import listener
from typing import Optional

import config
import colour
import motors
import camera
import cv2
import evacuation_zone
import gyroscope
import led

integral, derivative, last_error = 0, 0, 0
last_angle = 90

min_integral_reset_count, integral_reset_count = 2, 0
running_error = [0 for _ in range(0, 25)]

main_loop_count = 0
silver_loop_count = 0
min_green_loop_count = 20
gap_loop_count = 0
acute_loop_count = 0

uphill_trigger   = False
downhill_trigger = False
camera_enable = False

def follow_line() -> None:
    global main_loop_count, silver_loop_count, acute_loop_count, integral
    global camera_enable

    while True:
        if listener.get_mode() != 1: break
        
        colour_values = colour.read()
        gyro_values = gyroscope.read()
        
        if not camera_enable:
            green_signal = green_check(colour_values)
            silver_check(colour_values)
            acute_signal = acute_check(colour_values)

        if len(green_signal) != 0 and main_loop_count > min_green_loop_count and acute_loop_count == 0 and not camera_enable:
            integral = 0
            intersection_handling(green_signal, colour_values)
        elif silver_loop_count >= 20 and camera_enable:
            silver_loop_count = 0
            listener.mode = 2
            evacuation_zone.main()
            
            for i in range(100):
                colour_values = colour.read()
                gyro_values = gyroscope.read()
                PID("PID EVAC", colour_values, gyro_values, 0.32, 0.003, 0)
                
        elif gap_check(colour_values):
            motors.run(0, 0)
            gap_handling(1)
        else:
            if acute_loop_count > 0:
                PID("PID ACUTE", colour_values, gyro_values, 0.65, 0.008, 0)
                acute_loop_count -= 1
                
                if len(green_signal) != 0 and abs(integral) <= 500 and green_signal != "+ pass": acute_loop_count = 0
            else:
                PID("PID", colour_values, gyro_values, 0.45, 0.0011, -2)

        main_loop_count = main_loop_count + 1 if main_loop_count < 2**31 - 1 else 0
        
        print()
        
def PID(text: str, colour_values: list[int], gyro_values: list[Optional[int]], kP: float, kI: float, kD: float) -> None:
    global main_loop_count, last_green_loop, running_error, acute_loop_count
    global integral, derivative, last_error, integral_reset_count
    global uphill_trigger, downhill_trigger, camera_enable
    
    speed = config.line_base_speed
    
    if gyro_values[0] is not None:
        uphill_trigger   = True if gyro_values[0] >=  35 else False
        downhill_trigger = True if gyro_values[0] <= -35 else False
    
    if uphill_trigger or downhill_trigger or "ACUTE" in text or "EVAC" in text:
        led.on()
        if uphill_trigger:
            camera_line_follow(0.5, 30)
            text += " RAMP UP"
            camera_enable = True
        elif downhill_trigger:
            camera_line_follow(0.5, 7)
            text += " RAMP DOWN"
            camera_enable = True
        elif "ACUTE" in text:
            motors.pause()
        else:
            camera_line_follow(0.8, speed)
        
    else:
        led.off()
        camera_enable = False
    
        outer_error = config.outer_multi * (colour_values[0] - colour_values[4])
        inner_error = config.inner_multi * (colour_values[1] - colour_values[3])
        total_error = outer_error + inner_error
        
        running_error[main_loop_count % 25] = total_error
        
        integral_reset_count = integral_reset_count + 1 if abs(total_error) <= 15 and colour_values[1] >= 15 and colour_values[3] >= 15 else 0
        
        if integral_reset_count >= min_integral_reset_count: integral = 0
        elif integral <= -20000: integral = -19999
        elif integral >=  20000: integral =  19999
        else: integral += total_error if abs(total_error) > 100 else total_error ** 2 * 0.00005
        derivative = total_error - last_error
    
        turn = total_error * kP + integral * kI + derivative * kD
        v1, v2 = speed + turn, speed - turn
    
        motors.run(v1, v2)
        last_error = total_error
        
        config.update_log([f"{text}", f"{main_loop_count}", ", ".join(list(map(str, colour_values))), f"{total_error:.2f} {integral:.2f} {derivative:.2f}", f"{v1:.2f} {v2:.2f}"], [24, 8, 30, 16, 10])

def camera_line_follow(kP: float, speed: int) -> int:
    global last_angle
    
    image = camera.capture_array()
    transformed_image = camera.perspective_transform(image)
    line_image = transformed_image.copy()
    
    black_contour, black_image = camera.find_line_black_mask(transformed_image, line_image, last_angle)
    angle = camera.calculate_angle(black_contour, line_image, last_angle)
    
    error = 90 - angle
    turn = error * kP

    v1, v2 = speed - turn, speed + turn

    motors.run(v1, v2)
    
    last_angle = angle
    if config.X11: cv2.imshow("Line", line_image)
    config.update_log([
            "Camera Line Follow", 
            f"{angle:.2f}",
            f"{error:.2f}",
            f"{turn:.2f}",
            f"{v1:.2f} {v2:.2f}"
        ],
        [24, 8, 30, 16, 10]
    )

def green_check(colour_values: list[int]) -> str:
    global integral, main_loop_count
    signal = ""    

    if colour_values[0] <= 20 and colour_values[1] <= 20 and colour_values[3] <= 20 and colour_values[4] <= 20 and colour_values[2] <= 40 and abs(integral) <= 1500:
        if colour_values[5] > 35 and colour_values[6] > 35:
            signal = "+ pass"
        else: 
            if colour_values[5] <= 35 and colour_values[6] <= 35: signal = "+ double"
            elif colour_values[5] <= 35: signal = "+ left"
            elif colour_values[6] <= 35: signal = "+ right"
            else: pass

    else:
        outer_values = [colour_values[0], colour_values[4]]
        outer_values.sort()      
        inner_values = [colour_values[1], colour_values[3]]
        inner_values.sort()
  
        outer_left_enable  = True if outer_values[0] == colour_values[0] and outer_values[0] < 27 else False
        outer_right_enable = True if outer_values[0] == colour_values[4] and outer_values[0] < 27 else False
        inner_left_enable  = True if inner_values[0] == colour_values[1] and inner_values[0] < 15 else False
        inner_right_enable = True if inner_values[0] == colour_values[3] and inner_values[0] < 15 else False
        
        if (outer_left_enable or outer_right_enable or inner_left_enable or inner_right_enable) and colour_values[2] >= 40 and colour_values[4] <= 30 and colour_values[0] <= 30 and abs(integral) <= 6000:
            motors.run(0, 0, 0.1)
            new_colour_values = colour.read()
            
            if colour_values[0] > 30 or colour_values[4] > 30: signal = ""
            else:
                if (new_colour_values[5] <= 5 or new_colour_values[6] <= 5) and abs(new_colour_values[5] - new_colour_values[6]) >= 70:
                    signal = "|-" if new_colour_values[6] < new_colour_values[5] else "-|"
                else:
                    signal = "|-" if (colour_values[0] + colour_values[1]) >= (colour_values[3] + colour_values[4]) else "-|"
                    
                real_fake = " fake" if colour_values[5] + colour_values[6] >= 120 else " real"
                
                if signal == "|-" and colour_values[6] > 70 and real_fake == " real": real_fake = " fake"
                if signal == "-|" and colour_values[5] > 70 and real_fake == " real": real_fake = " fake"
                
                signal += real_fake

    if len(signal) != 0:
        config.update_log(["GREEN CHECK", ", ".join(list(map(str, colour_values))), f"{signal}"], [24, 30, 10])
        print()
    
    return signal
        
def intersection_handling(signal: str, colour_values) -> None:
    global main_loop_count
    main_loop_count = 0
    
    config.update_log([f"INTERSECTION HANDLING", f"{signal}"], [24, 16])
    print()
    
    if signal == "+ left":
        motors.run      ( config.line_base_speed * 0.9,  config.line_base_speed * 1.1, 0.35)
        motors.run      (-config.line_base_speed * 0.9,  config.line_base_speed * 1.1, 1.85)
        motors.run_until(-config.line_base_speed * 0.9,  config.line_base_speed * 1.1, colour.read, 3, ">=", 23, "[R] ALIGNING >= 23")
        motors.run_until(-config.line_base_speed * 0.9,  config.line_base_speed * 1.1, colour.read, 3, "<=", 45, "[R] ALIGNING <= 45")
        motors.run      ( config.line_base_speed * 0.9, -config.line_base_speed * 1.1, 0.35)

    elif signal == "+ double":
        motors.run      (-config.line_base_speed * 2  ,   config.line_base_speed * 2 , 1.65)
        motors.run_until(-config.line_base_speed * 2  ,   config.line_base_speed * 2 , colour.read, 3, "<=", 45, "[R] ALIGNING <= 45")
        motors.run      ( config.line_base_speed * 2  ,  -config.line_base_speed * 2 , 0.2)
        
    elif signal == "+ pass":
        motors.run      ( config.line_base_speed      ,  config.line_base_speed      , 0.6)
    
    elif signal == "+ right":
        motors.run      ( config.line_base_speed      ,  config.line_base_speed      , 0.35)
        motors.run      ( config.line_base_speed      , -config.line_base_speed      , 1.85)
        motors.run_until( config.line_base_speed      , -config.line_base_speed      , colour.read, 1, ">=", 23, "[L] ALIGNING >= 23")
        motors.run_until( config.line_base_speed      , -config.line_base_speed      , colour.read, 1, "<=", 45, "[L] ALIGNING <= 45")
        motors.run      (-config.line_base_speed      ,  config.line_base_speed      , 0.35)
    
    elif signal == "-| real":
        motors.run      (-config.line_base_speed      , -config.line_base_speed      , 0.2)
        motors.run      (-config.line_base_speed      , -config.line_base_speed      , 0.2)
        motors.run_until( config.line_base_speed      ,  config.line_base_speed      , colour.read, 1, "<=", 20, "[L] FORWARDS <= 10")
        motors.run      ( config.line_base_speed      ,  config.line_base_speed      , 0.3)
        motors.run      (-config.line_base_speed      ,  config.line_base_speed      , 0.8)
        motors.run_until(-config.line_base_speed      ,  config.line_base_speed      , colour.read, 3, "<=", 45, "[R] ALIGNING <= 30")
        motors.run_until(-config.line_base_speed      ,  config.line_base_speed      , colour.read, 3, ">=", 23, "[R] ALIGNING >= 23")
        motors.run      ( config.line_base_speed      , -config.line_base_speed      , 0.35)
  
    elif signal == "-| fake":
        motors.run_until( config.line_base_speed      , -config.line_base_speed      , colour.read, 3,  ">=", 23, "[R] ALIGNING >= 23")
        motors.run      ( config.line_base_speed      , -config.line_base_speed      , 0.3)
        motors.run      ( config.line_base_speed      ,  config.line_base_speed      , 0.7)
 
    elif signal == "|- real":
        motors.run      (-config.line_base_speed      , -config.line_base_speed      , 0.2)
        motors.run_until( config.line_base_speed      ,  config.line_base_speed      , colour.read, 3, "<=", 20, "[R] FORWARDS <= 10")
        motors.run      ( config.line_base_speed      ,  config.line_base_speed      , 0.3)
        motors.run      ( config.line_base_speed      , -config.line_base_speed      , 0.8)
        motors.run_until( config.line_base_speed      , -config.line_base_speed      , colour.read, 1, ">=", 23, "[L] ALIGNING >= 23")
        motors.run_until( config.line_base_speed      , -config.line_base_speed      , colour.read, 1, "<=", 45, "[L] ALIGNING <= 30")
        motors.run      (-config.line_base_speed      ,  config.line_base_speed      , 0.35)
  
    elif signal == "|- fake":
        motors.run_until(-config.line_base_speed      ,  config.line_base_speed      , colour.read, 1,  ">=", 23, "[L] ALIGNING >= 23")
        motors.run      (-config.line_base_speed      ,  config.line_base_speed      , 0.3)
        motors.run      ( config.line_base_speed      ,  config.line_base_speed      , 0.7)
    
    else:
        print(signal)
        
def acute_check(colour_values: list[int]) -> None:
    global acute_loop_count
    right_acute = colour_values[6] <= 20 and colour_values[5] <= 70
    left_acute  = colour_values[5] <= 20 and colour_values[6] <= 70 
    
    signal = ""
    
    if (left_acute or right_acute) and colour_values[2] >= 80: signal = "acute"
    
    if len(signal) != 0:
        while True:
            colour_values = colour.read()
            
            if colour_values[5] < 50 and colour_values[6] < 50: motors.run(-config.line_base_speed, -config.line_base_speed)
            elif colour_values[5] < 50:                         motors.run(-config.line_base_speed, 0)
            else:                                               motors.run(0, -config.line_base_speed)
            
            if colour_values[2] <= 30: break
        
        motors.run(-config.line_base_speed, -config.line_base_speed, 0.8)
        acute_loop_count = 250

        config.update_log([f"PID ACUTE CHECK", ", ".join(list(map(str, colour_values))), f"{signal}"], [24, 30, 10])
    
    return signal
        
def silver_check(colour_values: list[int]) -> str:
    global silver_loop_count
    
    silver_colour_check = [1 if value >= 130 else 0 for value in colour_values[0:5]]
    if sum(silver_colour_check) > 0: silver_loop_count += 1
    else: silver_loop_count = 0
    
def gap_check(colour_values: list[int]) -> bool:
    global gap_loop_count
    
    white_values = [1 if value > 90 else 0 for value in colour_values]
    gap_loop_count = gap_loop_count + 1 if sum(white_values) == 7 else 0
    
    return True if gap_loop_count > 20 else False

def gap_handling(kP: float) -> None:
    global last_angle
    led.on()
    image = camera.capture_array()
    motors.run(0, 0, 0.5)
    
    while True:
        image = camera.capture_array()
        transformed_image = camera.perspective_transform(image)
        line_image = transformed_image.copy()
        
        colour_values = colour.read()
        if colour_values[2] < 50: break
        
        black_contour, black_image = camera.find_line_black_mask(transformed_image, line_image, last_angle)
        angle = camera.calculate_angle(black_contour, line_image, last_angle)
        
        error = 90 - angle
        turn = error * kP
        motors.run(config.line_base_speed - turn, config.line_base_speed + turn)
        
        last_angle = angle
        if config.X11: cv2.imshow("Line", line_image)
        
        config.update_log(["GAP HANDLING", f"{error} {angle}"], [24, 18])
        print()
