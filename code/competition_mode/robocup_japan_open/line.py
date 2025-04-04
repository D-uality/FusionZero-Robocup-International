import os
import sys
import time
from listener import listener

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import time
from listener import listener
from typing import Optional
import operator

import config
import colour
import motors
import camera
import touch_sensors
import laser_sensors
import cv2
import evacuation_zone
import gyroscope
import led
import oled_display
import random
import green_functions
import numpy as np

ir_integral = ir_derivative = ir_last_error = 0
camera_integral = camera_derivative = camera_last_error = 0
camera_last_angle = 90
last_angle = 90

min_integral_reset_count, integral_reset_count = 2, 0

main_loop_count = 0
last_yaw = 0
min_green_loop_count = 50
gap_loop_count = 0

silver_min = 140
silver_count = 0
red_threshold = [ [55, 75], [35, 65] ]
red_count = 0

# uphill_count = downhill_count = 0
touch_count = 0
laser_close_count = 0
last_uphill = 0

uphill_trigger = downhill_trigger = seasaw_trigger = evac_trigger = False
tilt_left_trigger = tilt_right_trigger = False
gap_trigger = False
evac_exited = False
camera_enable = False

def main(evacuation_zone_enable: bool = False) -> None:
    global main_loop_count, laser_close_count, silver_count, red_count, evac_trigger, evac_exited, touch_count, last_uphill

    green_signal = ""
    colour_values = colour.read()
    gyroscope_values = gyroscope.read()
    touch_values = touch_sensors.read()
    # laser_value = laser_sensors.read([config.x_shut_pins[1]])

    # if laser_value[0] is not None: laser_close_count = laser_close_count + 1 if laser_value[0] < 8 and laser_value[0] != 0 else 0

    seasaw_check()
    red_check()
    silver_count = silver_check(colour_values, silver_count)

    if not camera_enable:
        touch_count = touch_count + 1 if sum(touch_values) != 2 else 0
        gap_check(colour_values)
        green_signal = green_check(colour_values)
        # if evac_exited: red_count = red_check(colour_values, red_count)
            
    if silver_count > 20:
        silver_count = 0

        oled_display.reset()
        oled_display.text("Silver Found", 0, 0, size=10)
        oled_display.text(",".join(map(str, colour_values)), 0, 12, size=10)
        evacuation_zone.main()
        main_loop_count = 0

        led.on()
        motors.run(0, 0, 2)

        evac_trigger = True
                    
    elif red_count > 2:
        motors.run(config.line_speed, config.line_speed, 1)
        red_count = 0
        oled_display.reset()
        oled_display.text("Red Found", 0, 0, size=10)
        oled_display.text(",".join(map(str, colour_values)), 0, 12, size=10)
        
        print("Red Found")
        motors.run(0, 0, 10)

    elif touch_count >= 300:
        avoid_obstacle()
        touch_count = 0
    
    elif len(green_signal) > 0 and not camera_enable:
        intersection_handling(green_signal, colour_values)
        
    else:
        follow_line(colour_values, gyroscope_values)

    print()
    main_loop_count += 1

def follow_line(colour_values: list[int], gyroscope_values: list[Optional[int]]) -> None:
    global uphill_trigger, downhill_trigger, tilt_left_trigger, tilt_right_trigger, gap_trigger, seasaw_trigger, evac_trigger
    global main_loop_count, last_yaw, last_uphill, touch_count
    global ir_integral, ir_derivative, ir_last_error, integral_reset_count, min_integral_reset_count
    global camera_integral, camera_derivative, camera_last_error, camera_last_angle
    global camera_enable
    
    # Finding modifiers
    # ramp detection
    if gyroscope_values[0] is not None:
        
        if uphill_trigger == True and not gyroscope_values[0] >= 15 and main_loop_count >= 100:
            print("reseting main loop")
            main_loop_count = -200
            uphill_trigger = False
            
        if not uphill_trigger and gyroscope_values[0] >= 15 and main_loop_count >= 100:
            print("moving back!")
            motors.run(-config.line_speed, -config.line_speed, 1.5)
            uphill_trigger = True
            main_loop_count = -100
        
        if downhill_trigger == True and not gyroscope_values[0] <= -15 and main_loop_count >= 100:
            print("reseting main loop")
            main_loop_count = -200
            downhill_trigger = False
            
        # uphill_trigger   = True if gyroscope_values[0] >=  15 and main_loop_count >= 100 else False
        downhill_trigger = True if gyroscope_values[0] <= -15 else False
        
    # tilt detection
    if gyroscope_values[1] is not None:
        tilt_left_trigger  = True if gyroscope_values[1] >=  15 else False
        tilt_right_trigger = True if gyroscope_values[1] <= -15 else False

    # RESET DETECTION
    if colour_values[2] <= 40 and abs(camera_last_error) <= 20 and abs(camera_integral) <= 20:  
        if gap_trigger    and main_loop_count > 80: gap_trigger    = False
        if seasaw_trigger and main_loop_count > 150: seasaw_trigger = False
    
    if gap_trigger and main_loop_count > 120:
        main_loop_count = 0
        gap_trigger = False
    
    if evac_trigger:
        if main_loop_count > 75: evac_trigger = False
        
    if tilt_left_trigger or tilt_right_trigger:
        uphill_trigger = False
        
    if seasaw_trigger:
        uphill_trigger = downhill_trigger = False

    # Modifiers
    modifiers = []
    if uphill_trigger:      modifiers.append("UPHILL")
    if downhill_trigger:    modifiers.append("DOWNHILL")
    if tilt_left_trigger:   modifiers.append("TILT LEFT")
    if tilt_right_trigger:  modifiers.append("TILT RIGHT")
    if gap_trigger:         modifiers.append("GAP")
    if seasaw_trigger:      modifiers.append("SEASAW")
    if evac_trigger:        modifiers.append("EVAC")
    modifiers = " ".join(modifiers)

    # Counting Modifiers
    last_uphill = 0 if uphill_trigger else last_uphill + 1

    # PID Values
    if   uphill_trigger:   kP, kI, kD, v = 1 , 0     ,  0  , 25
    elif downhill_trigger: kP, kI, kD, v = 0.5 , 0     ,  0  , 10
    elif gap_trigger:      kP, kI, kD, v = 1   , 0     ,  0  , config.line_speed
    elif seasaw_trigger:   kP, kI, kD, v = 1   , 0     ,  0  , config.line_speed
    elif evac_trigger:     kP, kI, kD, v = 0.5 , 0     ,  0  , 18
    else:                  kP, kI, kD, v = 1.35, 0     ,  0  , config.line_speed

    # Input method
    if uphill_trigger or downhill_trigger or gap_trigger or seasaw_trigger or evac_trigger or tilt_left_trigger or tilt_right_trigger:
        modifiers += " CAMERA"
        camera_enable = True
        led.on()

        image = camera.capture_array()
        transformed_image = camera.perspective_transform(image, modifiers)
        line_image = transformed_image.copy()
        
        black_contour, black_image = camera.find_line_black_mask(transformed_image, line_image, camera_last_angle)
        green_contours, green_image = green_functions.green_mask(transformed_image, line_image)

        valid_corners = []
        if len(green_contours) == 1:
            valid_rect, line_image = green_functions.validate_green_contour(green_contours[0], black_image, line_image)
            if valid_rect is not None:
                valid_corners.append(valid_rect)
        elif len(green_contours) > 1:
            valid_corners, line_image = green_functions.validate_multiple_contours(green_contours, black_image, line_image)

        if valid_corners is not None:
            green = green_functions.green_sign(valid_corners, black_image, line_image)
        else:
            green = "None"

        angle = camera.calculate_angle(black_contour, line_image, camera_last_angle)
                
        if green == "Left":
            if tilt_left_trigger:
                motors.run( config.line_speed,  config.line_speed, 2)
                motors.run_until(-config.line_speed, config.line_speed * 1.1, colour.read, 2, ">=", 40, "FIRST ALIGN")
                motors.run(0, 0, 0.15)
                motors.run(-config.line_speed, config.line_speed, 1.2)
                motors.run(0, 0, 0.15)
                motors.run_until(-config.line_speed, config.line_speed, colour.read, 2, "<=", 30, "SECOND ALIGN")
                motors.run(0, 0, 0.15)
                motors.run(-config.line_speed, config.line_speed, 0.4)
                motors.run(0, 0, 0.15)   
            else:
                motors.run( config.line_speed,  config.line_speed, 1.7)
                motors.run_until(0, config.line_speed * 1.1, colour.read, 2, ">=", 40, "FIRST ALIGN")
                motors.run(0, 0, 0.15)
                motors.run(0, config.line_speed, 1.2)
                motors.run(0, 0, 0.15)
                motors.run_until(0, config.line_speed, colour.read, 2, "<=", 30, "SECOND ALIGN")
                motors.run(0, 0, 0.15)
            
        elif green == "Right":
            if tilt_right_trigger:
                motors.run( config.line_speed,  config.line_speed, 2)
                motors.run_until(config.line_speed * 1.1, -config.line_speed, colour.read, 2, ">=", 40, "FIRST ALIGN")
                motors.run(0, 0, 0.15)
                motors.run(config.line_speed, -config.line_speed, 1.2)
                motors.run(0, 0, 0.15)
                motors.run_until(config.line_speed, -config.line_speed, colour.read, 2, "<=", 30, "SECOND ALIGN")
                motors.run(0, 0, 0.15)
                motors.run(config.line_speed, -config.line_speed, 0.4)
                motors.run(0, 0, 0.15)   
            else:
                motors.run( config.line_speed,  config.line_speed, 1.7)
                motors.run_until(config.line_speed * 1.1, 0, colour.read, 2, ">=", 40, "FIRST ALIGN")
                motors.run(0, 0, 0.15)
                motors.run(config.line_speed, 0, 1.2)
                motors.run(0, 0, 0.15)
                motors.run_until(config.line_speed, 0, colour.read, 2, "<=", 30, "SECOND ALIGN")
                motors.run(0, 0, 0.15)
                
                
            
        else:
            error = angle - 90

            if abs(error) < 10: camera_integral = 0
            camera_integral += error if abs(error) > 5 else 0
            camera_derivative = error - camera_last_error

            turn = error * kP + camera_integral * kI + camera_derivative * kD
            v1, v2 = v + turn, v - turn

            motors.run(v1, v2)
            camera_last_angle = angle
            camera_last_error = error
            config.update_log([modifiers+" PID", f"{main_loop_count}", f"{angle:.2f} {colour_values[2]}", f"{error:.2f} {camera_integral:.2f} {camera_derivative:.2f}", f"{v1:.2f} {v2:.2f}", f"{silver_count} {laser_close_count} {touch_count} {red_count}"], [24, 8, 30, 30, 10, 30])
        
        if config.X11: cv2.imshow("line", line_image)
        
    else:
        camera_enable = False
        led.on()
        
        updated_colour_values = [min(value, 100) for value in colour_values]

        outer_error = 1 * (updated_colour_values[0] - updated_colour_values[4])
        inner_error = 1 * (updated_colour_values[1] - updated_colour_values[3])
        error = outer_error + inner_error
                
        integral_reset_count = integral_reset_count + 1 if abs(error) <= 15 and updated_colour_values[1] + updated_colour_values[3] >= 60 else 0
        
        if integral_reset_count >= min_integral_reset_count: ir_integral = 0
        elif ir_integral <= -50000: ir_integral = -49999
        elif ir_integral >=  50000: ir_integral =  49999
        else: ir_integral += error * 0.8 if abs(error) > 120 else error * 0.2
        ir_derivative = error - ir_last_error

        if updated_colour_values[0] > 70 and updated_colour_values[1] > 70 and updated_colour_values[3] > 70 and updated_colour_values[4] > 70:
            middle_multi = 0.2
        else:
            middle_multi = max(1 + (updated_colour_values[2] - 85)/100, 0)

        turn = middle_multi * (error * kP + ir_integral * kI + ir_derivative * kD)
        v1, v2 = v + turn, v - turn

        # if   tilt_left_trigger:  v1, v2 = v1 + 5, v2 - 5
        # elif tilt_right_trigger: v1, v2 = v1 - 5, v2 + 5
    
        # Run slower if touching
        if touch_count > 20: v1, v2 = 0.7 * v1, 0.7 * v2

        motors.run(v1, v2)
        ir_last_error = error
        last_yaw = gyroscope_values[2] if ir_integral == 0 and gyroscope_values[2] is not None else last_yaw
                
        config.update_log([modifiers+" PID", f"{main_loop_count}", ", ".join(list(map(str, colour_values))), f"{error:.2f} {ir_integral:.2f} {ir_derivative:.2f}", f"{v1:.2f} {v2:.2f}", f"{silver_count} {laser_close_count} {touch_count} {red_count}"], [24, 8, 30, 30, 10, 30])    
        
def green_check(colour_values: list[int]) -> str:
    global main_loop_count, ir_integral
    signal = ""

    if main_loop_count < 20: return signal

    # Black Types
    # Left Mid
    left_black = colour_values[0] < 20 and colour_values[1] < 20

    # Right Mid
    right_black = colour_values[3] < 20 and colour_values[4] < 20
    # Double
    left_double_black  = left_black  and (colour_values[3] < 35 or colour_values[4] < 50)
    right_double_black = right_black and (colour_values[0] < 35 or colour_values[1] < 50)

    # Green
    if colour_values[0] < 30 and colour_values[1] < 30 and colour_values[3] < 30 and colour_values[4] < 30 and colour_values[2] > 40 and colour_values[5] + colour_values[6] >= 120:
        if colour_values[0] + colour_values[1] < colour_values[3] + colour_values[4]:
            signal = "fake left"
        else:
            signal = "fake right"
    elif colour_values[2] <= 80 and abs(ir_integral) <= 1300:
        if (left_double_black or right_double_black) and ((colour_values[5] <= 42 and colour_values[6] <= 52) or (colour_values[5] <= 52 and colour_values[6] <= 42)):
            signal = "double"
            
        elif (left_black or left_double_black) and colour_values[5] <= 55:
            signal = "left"
        
        elif (right_black or right_double_black) and colour_values[6] <= 55:
            signal = "right"
    
    # if colour_values[5] <= 5 and colour_values[6] <= 5 and ir_integral <= 1000:
    #     motors.run(-config.line_speed, -config.line_speed, 0.1)
        
    #     new_colour_values = colour.read()
    #     main_loop_count = 0
    #     config.update_log(["GREEN NEW VALUES", ",".join(list(map(str, new_colour_values)))], [24, 30])
        
    #     if new_colour_values[0] <= 30 and new_colour_values[1] <= 30 and new_colour_values[3] <= 30 and new_colour_values[4] <= 30:
    #         signal = "double"
        
    # else:             
    #     if colour_values[5] <= 0:
            
    #         motors.run(-config.line_speed, -config.line_speed, 0.1)
            
    #         # new_colour_values = colour.read()
    #         # config.update_log(["GREEN NEW VALUES", ",".join(list(map(str, new_colour_values)))], [24, 30])
            
    #         # if new_colour_values[0] <= 30 and new_colour_values[1] <= 30:
    #         signal = "left"
                    
    #     elif colour_values[6] <= 0:
    #         motors.run(-config.line_speed, -config.line_speed, 0.1)
            
    #         # new_colour_values = colour.read()
    #         # config.update_log(["GREEN NEW VALUES", ",".join(list(map(str, new_colour_values)))], [24, 30])
            
    #         # if new_colour_values[3] <= 30 and new_colour_values[4] <= 30:
    #         signal = "right"
        
    if len(signal) != 0: config.update_log(["GREEN CHECK", ",".join(list(map(str, colour_values))), signal], [24, 30, 10])

    return signal

def intersection_handling(signal: str, colour_values) -> None:
    global main_loop_count, last_yaw, ir_integral
    main_loop_count = ir_integral = turn_count = 0
    
    # OLED update for green signal:
    oled_display.reset()
    oled_display.text(f"Green: {signal}", 0, 0, size=10)
    oled_display.text(f"{','.join(map(str, colour_values))}", 0, 10, size=10)

    config.update_log([f"INTERSECTION HANDLING", f"{signal}"], [24, 16])
    print()
    
    while True:
        current_angle = gyroscope.read()[2]
        if current_angle is not None: break

    if signal == "left":
        motors.run(config.line_speed * 1, config.line_speed * 1, 0.1)
        motors.run_until(-config.line_speed, config.line_speed * 1.1, colour.read, 2, ">=", 50, "FIRST ALIGN")
        motors.run(0, 0, 0.15)
        motors.run(-config.line_speed, config.line_speed, 0.15)
        motors.run(0, 0, 0.15)
        motors.run_until(-config.line_speed, config.line_speed, colour.read, 2, "<=", 40, "SECOND ALIGN")
        motors.run(0, 0, 0.15)
        motors.run(-config.line_speed, config.line_speed, 0.35)
        motors.run(0, 0, 0.15)
        motors.run(config.line_speed, config.line_speed, 0.3)
        motors.run(0, 0, 0.15)

    elif signal == "right":
        motors.run(config.line_speed * 1, config.line_speed * 1, 0.1)
        motors.run_until(config.line_speed * 1.1, -config.line_speed, colour.read, 2, ">=", 50, "FIRST ALIGN")
        motors.run(0, 0, 0.15)
        motors.run(config.line_speed, -config.line_speed, 0.15)
        motors.run(0, 0, 0.15)
        motors.run_until(config.line_speed, -config.line_speed, colour.read, 2, "<=", 40, "SECOND ALIGN")
        motors.run(0, 0, 0.15)
        motors.run(config.line_speed, -config.line_speed, 0.35)
        motors.run(0, 0, 0.15)
        motors.run(config.line_speed, config.line_speed, 0.3)
        motors.run(0, 0, 0.15)
    
    elif signal == "double":
        motors.run(-config.line_speed - 10, config.line_speed + 10, 3)
        colour_values = colour.read()
        while colour_values[3] > 60 and turn_count < 1000:
            turn_count += 1
            colour_values = colour.read()

    elif signal == "fake left":
        angle = current_angle - 35 if abs(current_angle - last_yaw) < 20 else last_yaw
        motors.run_until( config.line_speed      , -config.line_speed      , gyroscope.read, 2, "<=", angle, "GYRO")
        motors.run      ( config.line_speed * 1.5,  config.line_speed * 1.5, 0.3)
                
    elif signal == "fake right":
        angle = current_angle + 35 if abs(current_angle - last_yaw) < 20 else last_yaw
        motors.run_until(-config.line_speed      ,  config.line_speed      , gyroscope.read, 2, ">=", angle, "GYRO")
        motors.run      ( config.line_speed * 1.5,  config.line_speed * 1.5, 0.3)
        
    else: print(signal)

def gap_check(colour_values: list[int]) -> None:
    global main_loop_count, gap_loop_count, gap_trigger, last_uphill

    white_values = [1 if value >= 70 and value <= silver_min else 0 for value in colour_values]
    gap_loop_count = gap_loop_count + 1 if sum(white_values) >= 5 and colour_values[2] >= 60 else 0

    gap_trigger = True if gap_loop_count >= 60 else False
    
    if gap_trigger:
        main_loop_count = 0
        motors.run(-config.line_speed * 1.5, -config.line_speed * 1.5, 0.5 if last_uphill < 100 else 0.2)
        motors.run(0, 0, 0.3)
        
        image = camera.capture_array()
        if config.X11: cv2.imshow("image", image)

def seasaw_check():
    global downhill_trigger, last_uphill, seasaw_trigger
    global main_loop_count
    
    if main_loop_count <= 20: return None
    
    if downhill_trigger and last_uphill < 40:
        last_uphill = 100
        main_loop_count = 0
        motors.run(0, 0, 2)
        
        start_time = time.time()
        while True:
            colour_values = colour.read()
            
            motors.run(-config.line_speed, -config.line_speed)
            
            if colour_values[2] <= 50:
                motors.run(-config.line_speed, -config.line_speed, 0.3)
                break
            
            if time.time() - start_time > 2:
                break
            
            config.update_log(["SEASAW", "MOVING BACK TILL BLACK"], [24, 30])
            print()
            
        # motors.run_until(-config.line_speed, -config.line_speed, colour.read, 2, "<=", 50, "MOVING BACK TILL BLACK")
        # motors.run(-config.line_speed, -config.line_speed, 0.3)
        seasaw_trigger = True

def avoid_obstacle() -> None:    
    side_values = laser_sensors.read([config.x_shut_pins[0], config.x_shut_pins[2]])


    # Immediately update OLED for obstacle detection
    oled_display.reset()
    oled_display.text("Obstacle Detected", 0, 0, size=10)

    config.update_log(["OBSTACLE", "FINDING SIDE", ", ".join(list(map(str, side_values)))], [24, 50, 14])
    print()

    # Clockwise if left > right, else random
    if side_values[0] <= 30 or side_values[1] <= 30:
        direction = "cw" if side_values[0] > side_values[1] else "ccw"
    else:
        direction = "cw" if random.randint(0, 1) == 0 else "ccw"

    # Turn until appropriate laser sees obstacle
    v1 = v2 = laser_pin = 0
    if direction == "cw":
        v1 = -config.line_speed
        v2 =  config.line_speed * 0.8
        laser_pin = 2
    else:
        v1 =  config.line_speed * 0.8
        v2 = -config.line_speed
        laser_pin = 0

    # SETUP

    motors.run(-config.line_speed, -config.line_speed, 0.5)

    # Over turn passed obstacle
    oled_display.text("Turning till obstacle", 0, 10, size=10)
    for i in range(25):
        motors.run_until(1.2 * v1, 1.2*v2, laser_sensors.read, laser_pin, "<=", 20, "TURNING TILL OBSTACLE")
        motors.run(1.2 * v1, 1.2 * v2, 0.01)
    
    oled_display.text("Turning past obstacle", 0, 20, size=10)
    for i in range(25):
        motors.run_until(1.2 * v1, 1.2*v2, laser_sensors.read, laser_pin, ">=", 20, "TURNING PAST OBSTACLE")
        motors.run(1.2 * v1, 1.2 * v2, 0.01)

    motors.run(1.2 * v1, 1.2*v2, 0.1)

    motors.run(0, 0, 1)

    # Turn back onto obstacle
    oled_display.text("Turning till obstacle", 0, 30, size=10)
    motors.run_until(-v1, -v2, laser_sensors.read, laser_pin, "<=", 15, "TURNING TILL OBSTACLE")
    motors.run(-1.2 * v1, -1.2 * v2, 1)

    # Circle obstacle
    v1 = v2 = laser_pin = colour_align_pin = 0

    if direction == "cw":
        v1 =  config.line_speed
        v2 = -config.line_speed
        laser_pin = 2
        colour_align_pin = 3
        colour_black_pin = 1
    else:
        v1 = -config.line_speed
        v2 =  config.line_speed
        laser_pin = 0
        colour_align_pin = 1
        colour_black_pin = 3

    initial_sequence = True
    oled_display.text("Circle Obstacle: Starting", 0, 40, size=10)
    circle_obstacle(config.line_speed, config.line_speed, laser_pin, colour_black_pin, "<=", 13, "FORWARDS TILL OBSTACLE")
    motors.run(config.line_speed, config.line_speed, 0.5)

    while True:
        oled_display.reset()
        oled_display.text("Circle: Forwards till not", 0, 0, size=10)
        if circle_obstacle(config.line_speed, config.line_speed, laser_pin, colour_black_pin, ">=", 20, "FORWARDS TILL NOT OBSTACLE"): pass
        elif not initial_sequence: break
        # motors.run(-config.line_speed, -config.line_speed, 0.25)
        motors.run(0, 0, 0.15)

        oled_display.text("Turning till obstacle", 0, 10, size=10)
        if circle_obstacle(v1, v2, laser_pin, colour_black_pin, "<=", 15, "TURNING TILL OBSTACLE"): pass
        elif not initial_sequence: break
        motors.run(0, 0, 0.15)

        oled_display.text("Turning till not", 0, 20, size=10)
        if circle_obstacle(v1, v2, laser_pin, colour_black_pin, ">=", 18, "TURNING TILL NOT OBSTACLE"): pass
        elif not initial_sequence: break
        motors.run(v1, v2, 0.1)
        motors.run(0, 0, 0.15)

        oled_display.text("Forwards till obstacle", 0, 30, size=10)
        if circle_obstacle(config.line_speed, config.line_speed, laser_pin, colour_black_pin, "<=", 18, "FORWARDS TILL OBSTACLE"): pass
        elif not initial_sequence: break
        motors.run(0, 0, 0.15)

        initial_sequence = False
    
    oled_display.reset()
    oled_display.text("Black Found", 0, 0, size=10)


    config.update_log(["OBSTACLE", "FOUND BLACK"], [24, 50])
    print()
    
    # Turn in the opposite direction
    motors.run(config.line_speed, config.line_speed, 0.5)
    motors.run_until(config.line_speed, config.line_speed, colour.read, colour_align_pin, ">=", 60, "FORWARDS TILL WHITE")
    motors.run_until(-v1, -v2, colour.read, colour_align_pin, "<=", 50, "TURNING TILL BLACK")

def circle_obstacle(v1: float, v2: float, laser_pin: int, colour_pin: int, comparison: str, target_distance: float, text: str = "") -> bool:
    if   comparison == "<=": comparison_function = operator.le
    elif comparison == ">=": comparison_function = operator.ge

    while True:
        laser_value = laser_sensors.read([config.x_shut_pins[laser_pin]])[0]
        colour_values = colour.read()

        config.update_log(["OBSTACLE", text, f"{laser_value}"], [24, 50, 10])
        print()

        motors.run(v1, v2)
        if laser_value is not None:
            if comparison_function(laser_value, target_distance) and laser_value != 0: return True

        if colour_values[colour_pin] <= 30:
            return False

def silver_check(colour_values, silver_count):
    global silver_min, ir_integral
    if ir_integral >= 1000: return silver_count
    
    if any(colour_values[i] > silver_min for i in [0, 1, 3, 4]) and colour_values[2] > 60:
        silver_count += 1
    else: 
        silver_count = 0
    
    return silver_count

# def red_check(colour_values, red_count):
#     global red_threshold, main_loop_count
    
#     if main_loop_count < 50: return red_count

#     if (red_threshold[0][0] < colour_values[5] < red_threshold[0][1] or red_threshold[1][0] < colour_values[6] < red_threshold[1][1]) and all(colour_values[i] > 70 and colour_values[i] < 120 for i in range(5)):
#         red_count += 1
#     else: 
#         red_count = 0
    
#     return red_count

def red_check() -> None:
    global main_loop_count
    global camera_enable
    global red_count

    # Only process images every 10 loops
    
    if main_loop_count % 10 != 0:
        return None
    
    image = camera.capture_array()
    image = camera.perspective_transform(image, "LINE")
    if config.X11: cv2.imshow("image", image)
    
    # mask = cv2.inRange(image, (150, 0, 0), (255, 40, 40))
    mask = cv2.inRange(image, (0, 0, 120), (40, 40, 255))
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
    
    if config.X11: cv2.imshow("mask", mask)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        red_count = 0
        return None
    
    # Find valid contours based on criteria
    valid_contours = []
    
    for contour in contours:
        _, _, w, h = cv2.boundingRect(contour)

        if (   cv2.contourArea(contour) >= 100
            and w > h                         ):
            valid_contours.append(contour)
    
    if not valid_contours:
        red_count = 0
        return None
    
    red_count += 1
        
if __name__ == "__main__":
    camera.initialise("line")
    led.on()
        
    while True:
        camera_enable = False
        # camera.capture_array()
        red_check()
        
        print(main_loop_count, red_count)
        
        main_loop_count += 1