import time
from listener import listener
from typing import Optional

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

ir_integral = ir_derivative = ir_last_error = 0
camera_integral = camera_derivative = camera_last_error = camera_last_angle= 0
last_angle = 90

min_integral_reset_count, integral_reset_count = 2, 0

main_loop_count = 0
last_yaw = 0
min_green_loop_count = 50
gap_loop_count = 0

silver_min = 110
silver_count = 0

laser_close_count = 0
last_uphill = 0

uphill_trigger  = downhill_trigger = False
tilt_left_trigger = tilt_right_trigger = False
acute_trigger = gap_trigger = False

camera_enable = False

def main(evacuation_zone_enable: bool = False) -> None:
    global main_loop_count, laser_close_count, silver_count

    green_signal = ""
    colour_values = colour.read()
    gyroscope_values = gyroscope.read()
    touch_values = touch_sensors.read()
    laser_value = laser_sensors.read([config.x_shut_pins[1]])
    seasaw_check()

    if laser_value[0] < 10 and laser_value[0] != 0:
        laser_close_count += 1
    else:
        laser_close_count = 0

    print(touch_values)

    if not camera_enable:
    #     acute_check(colour_values)
    #     gap_check(colour_values)
        green_signal = green_check(colour_values)

        silver_count = silver_check(colour_values, silver_count)
    if silver_count > 10:
        print("Silver Found")
        motors.run(0, 0, 1)

        evacuation_zone.main()

    elif  touch_values[0] == 0 or touch_values[1] == 0 or laser_close_count > 15:
        avoid_obstacle()
    elif len(green_signal) > 0:
        intersection_handling(green_signal, colour_values)
    else:
        follow_line(colour_values, gyroscope_values)

    print()
    main_loop_count += 1

def follow_line(colour_values: list[int], gyroscope_values: list[Optional[int]]) -> None:
    global uphill_trigger, downhill_trigger, acute_trigger, tilt_left_trigger, tilt_right_trigger, gap_trigger
    global main_loop_count, last_yaw, last_uphill
    global ir_integral, ir_derivative, ir_last_error, integral_reset_count, min_integral_reset_count
    global camera_integral, camera_derivative, camera_last_error, camera_last_angle
    global camera_enable
    
    # Finding modifiers
    # ramp detection
    if gyroscope_values[0] is not None:
        uphill_trigger   = True if gyroscope_values[0] >=  15 else False
        downhill_trigger = True if gyroscope_values[0] <= -15 else False
    
    # tilt detection
    # if gyroscope_values[1] is not None:
    #     tilt_left_trigger  = True if gyroscope_values[1] >=  15 else False
    #     tilt_right_trigger = True if gyroscope_values[1] <= -15 else False

    # acute and gap RESET DETECTION
    if colour_values[2] <= 40 and abs(camera_last_error) < 15 and abs(camera_integral) == 0:
        if acute_trigger and main_loop_count > 150:
            # regain line
            # start_time = time.time()
            # while True:
            #     colour_values = colour.read()
            #     if   colour_values[5] < 20 and colour_values[6] >= 20: motors.run(config.line_speed, 0)
            #     elif colour_values[6] < 20 and colour_values[5] >= 20: motors.run(0, config.line_speed)
            #     else:                                                  motors.run(config.line_speed, config.line_speed)
                
            #     if time.time() - start_time > 2: break
            #     if colour_values[5] < 20 and colour_values[6] < 20: break
                
            #     config.update_log(["REGAINING LINE", f"{time.time() - start_time:.2f}", ", ".join(list(map(str, colour_values)))], [24, 10, 30])
            #     print()
                
            acute_trigger = False
            
        if gap_trigger and main_loop_count > 80:
            gap_trigger = False

    # Modifiers
    modifiers = []
    if uphill_trigger:      modifiers.append("UPHILL")
    if downhill_trigger:    modifiers.append("DOWNHILL")
    if acute_trigger:       modifiers.append("ACUTE")
    if tilt_left_trigger:   modifiers.append("TILT LEFT")
    if tilt_right_trigger:  modifiers.append("TILT RIGHT")
    if gap_trigger:         modifiers.append("GAP")
    modifiers = " ".join(modifiers)

    # Counting Modifiers
    last_uphill = 0 if uphill_trigger else last_uphill + 1

    # PID Values
    if   uphill_trigger:   kP, kI, kD, v = 0.4 , 0     ,  0  , 25
    elif downhill_trigger: kP, kI, kD, v = 0.5 , 0     ,  0  , 10
    elif acute_trigger:    kP, kI, kD, v = 1.5 , 0     ,  0  , 18
    elif gap_trigger:      kP, kI, kD, v = 1   , 0     ,  0  , config.line_speed
    # else:                  kP, kI, kD, v = 0.23, 0.005,  2  , config.line_speed
    else:                  kP, kI, kD, v = 1.5, 0, 0, config.line_speed

    # Define input method
    if uphill_trigger or downhill_trigger or acute_trigger or gap_trigger:
        modifiers += " CAMERA"
        camera_enable = True
        led.on()

        image = camera.capture_array()
        transformed_image = camera.perspective_transform(image)
        line_image = transformed_image.copy()
        
        black_contour, _ = camera.find_line_black_mask(transformed_image, line_image, camera_last_angle)
        angle = camera.calculate_angle(black_contour, line_image, camera_last_angle)
        
        error = angle - 90
        
        if abs(error) < 10: camera_integral = 0
        camera_integral += error if abs(error) > 5 else 0
        camera_derivative = error - camera_last_error

        turn = error * kP + camera_integral * kI + camera_derivative * kD
        v1, v2 = v + turn, v - turn

        if   tilt_left_trigger:  v1, v2 = v1 + 5, v2 - 5
        elif tilt_right_trigger: v1, v2 = v1 - 5, v2 + 5

        motors.run(v1, v2)
        camera_last_angle = angle
        camera_last_error = error
        
        if config.X11: cv2.imshow("line", line_image)
        config.update_log([modifiers+" PID", f"{main_loop_count}", f"{angle:.2f} {colour_values[2]}", f"{error:.2f} {camera_integral:.2f} {camera_derivative:.2f}", f"{v1:.2f} {v2:.2f}"], [24, 8, 30, 16, 10])
        
    else:
        camera_enable = False
        led.off()
    
        outer_error = 1 * (colour_values[0] - colour_values[4])
        inner_error = 1 * (colour_values[1] - colour_values[3])
        error = outer_error + inner_error
                
        integral_reset_count = integral_reset_count + 1 if abs(error) <= 15 and colour_values[1] + colour_values[3] >= 75 else 0
        
        if integral_reset_count >= min_integral_reset_count: ir_integral = 0
        elif ir_integral <= -50000: ir_integral = -49999
        elif ir_integral >=  50000: ir_integral =  49999
        else: ir_integral += error * 0.8 if abs(error) > 120 else error * 0.3
        ir_derivative = error - ir_last_error

        if colour_values[0] > 70 and colour_values[0] > 70 and colour_values[3] > 70 and colour_values[4] > 70:
            middle_multi = 0.2
        else:
            middle_multi = 1 + (colour_values[2] - 100)/100
        turn = middle_multi * (error * kP + ir_integral * kI + ir_derivative * kD)
        v1, v2 = v + turn, v - turn

        if   tilt_left_trigger:  v1, v2 = v1 + 5, v2 - 5
        elif tilt_right_trigger: v1, v2 = v1 - 5, v2 + 5
    
        motors.run(v1, v2)
        ir_last_error = error
        last_yaw = gyroscope_values[2] if ir_integral == 0 and gyroscope_values[2] is not None else last_yaw
        
        config.update_log([modifiers+" PID", f"{main_loop_count}", ", ".join(list(map(str, colour_values))), f"{error:.2f} {ir_integral:.2f} {ir_derivative:.2f}", f"{v1:.2f} {v2:.2f}"], [24, 8, 30, 16, 10])    
        
def green_check(colour_values: list[int]) -> str:
    global main_loop_count
    signal = ""

    if main_loop_count < 20: return signal

    # Black Types
    # Left Mid
    left_black = colour_values[0] < 30 and colour_values[1] < 30 and colour_values[2] < 50

    # Right Mid
    right_black = colour_values[3] < 30 and colour_values[4] < 30 and colour_values[2] < 50
    # Double
    left_double_black = left_black and (colour_values[3] < 40 or colour_values[4] < 40)
    right_double_black = right_black and (colour_values[0] < 40 or colour_values[1] < 40)

    # Green
    if (left_double_black or right_double_black) and colour_values[5] < 40 and colour_values[6] < 40:  signal = "double"
    elif (left_black or left_double_black) and colour_values[5] < 40: signal = "left"
    elif (right_black or right_double_black) and colour_values[6] < 40: signal = "right"


    return signal

def intersection_handling(signal: str, colour_values) -> None:
    global main_loop_count, last_yaw, ir_integral
    main_loop_count = ir_integral = turn_count = 0
    
    config.update_log([f"INTERSECTION HANDLING", f"{signal}"], [24, 16])
    print()
    
    
    while True:
        current_angle = gyroscope.read()[2]
        if current_angle is not None: break

    if signal == "left":
        motors.run(-config.line_speed - 5, config.line_speed + 5, 1.2)
        motors.run(config.line_speed, config.line_speed, 0.3)
        colour_values = colour.read()
        while colour_values[0] < 40 and turn_count < 500:
            turn_count += 1
            colour_values = colour.read()

    elif signal == "right":
        motors.run(config.line_speed + 5, -config.line_speed - 5, 1.2)
        motors.run(config.line_speed, config.line_speed, 0.3)
        colour_values = colour.read()
        while colour_values[4] < 40 and turn_count < 500:
            turn_count += 1
            colour_values = colour.read()
    
    elif signal == "double":
        motors.run(config.line_speed + 5, config.line_speed + 5, 0.2)
        motors.run(-config.line_speed - 10, config.line_speed + 10, 3)
        colour_values = colour.read()
        while colour_values[3] > 60 and turn_count < 1000:
            turn_count += 1
            colour_values = colour.read()
        
    else: print(signal)

def acute_check(colour_values: list[int]) -> None:
    global main_loop_count, acute_trigger, min_green_loop_count

    if main_loop_count < min_green_loop_count: return None
    
    if (
        ((colour_values[6] <= 20 and colour_values[5] <= 80) or (colour_values[6] <= 80 and colour_values[5] <= 20))
        and colour_values[2] >= 80
        and colour_values[0] + colour_values[4] >= 100
    ): acute_trigger = True
    
    if acute_trigger:
        config.update_log(["ACUTE CHECK", ", ".join(map(str, colour_values))], [24, 30])

        # while True:
        #     colour_values = colour.read()

        #     # if   colour_values[5] <= 50 and colour_values[6] <= 50: motors.run(-config.line_speed, -config.line_speed)
        #     # elif colour_values[5] <= 50:                            motors.run(-config.line_speed,  config.line_speed)
        #     # elif colour_values[6] <= 50:                            motors.run( config.line_speed, -config.line_speed)

        #     if colour_values[2] <= 20: break
        #     config.update_log(["ACUTE ORIENTATING", ", ".join(map(str, colour_values))], [24, 30])
        #     print()

        motors.run(-config.line_speed * 1.5, -config.line_speed * 1.5, 1.2)
        main_loop_count = 0

def gap_check(colour_values: list[int]) -> None:
    global main_loop_count, gap_loop_count, gap_trigger

    white_values = [1 if value > 90 else 0 for value in colour_values]
    gap_loop_count = gap_loop_count + 1 if sum(white_values) == 7 else 0

    gap_trigger = True if gap_loop_count >= 40 else False
    
    if gap_trigger:
        main_loop_count = 0
        motors.run(-config.line_speed * 1.5, -config.line_speed * 1.5, 1)
        motors.run(0, 0, 0.3)
        
        image = camera.capture_array()
        if config.X11: cv2.imshow("image", image)

def seasaw_check():
    global downhill_trigger, last_uphill
    
    if downhill_trigger and last_uphill < 40:
        motors.pause()

def circle_obstacle(laser_pin, turn_type, more_than, check_black, colour_is_black, min_move_time):
    laser_condition_met = False
    starting_time = time.time()
    current_time = 0

    while (not laser_condition_met or current_time < min_move_time) and (not colour_is_black[0] or not colour_is_black[1]):
        if turn_type == 'straight':
            motors.run(30, 30)
        elif turn_type == 'left':
            motors.run(25, -25)
        elif turn_type == 'right':
            motors.run(-25, 25)

        colour_values = colour.read()
        distances = laser_sensors.read()
        touch_values = touch_sensors.read()
        print(distances)

        if more_than == '>':
            laser_condition_met = distances[laser_pin] > 10 and distances[laser_pin] != 0
        elif more_than == '<':
            laser_condition_met = distances[laser_pin] < 10 and distances[laser_pin] != 0

        if check_black:
            colour_is_black[0] = (colour_values[1] < 50 and not colour_is_black[0]) or colour_is_black[0]
            colour_is_black[1] = (colour_values[3] < 50 and not colour_is_black[1]) or colour_is_black[1]

        for touch_side in range(1):
            if touch_values[touch_side] == 0 and turn_type == 'straight':
                if touch_side == 0:
                    motors.run(config.line_speed, -config.line_speed)
                if touch_side == 1:
                    motors.run(-config.line_speed, config.line_speed)

                colour.read()
                laser_sensors.read()
                touch_sensors.read()
                print()

        print(f"Laser Condition Met: {laser_condition_met}")

        current_time = time.time() - starting_time

    return colour_is_black

def avoid_obstacle():
    turn_count = 0
    print("Obstacle Detected")
    colour_is_black = [False, False]

    distances = laser_sensors.read()

    if distances[0] > distances[2]:
        print("Going Left")
        colour_is_black = circle_obstacle(2, 'right', '<', False, colour_is_black, 0.5)
        motors.run(config.line_speed, -config.line_speed, 0.3)
        colour_is_black = circle_obstacle(2, 'straight', '>', False, colour_is_black, 0.5)
        motors.run(-config.line_speed, -config.line_speed, 0.3)

        print("Left Obstacle Loop Phase")
        while not colour_is_black[0]:
            colour_is_black = circle_obstacle(2, 'left', '>', True, colour_is_black, 1)
            # motors.run(config.line_speed, -config.line_speed, 0.3)
            colour_is_black = circle_obstacle(2, 'straight', '>', True, colour_is_black, 1)
            motors.run(-config.line_speed, -config.line_speed, 0.3)

        motors.run(-config.line_speed - 5, config.line_speed + 5, 1.2)
        motors.run(config.line_speed, config.line_speed, 0.3)
        colour_values = colour.read()
        while colour_values[0] < 40 and turn_count < 500:
            turn_count += 1
            colour_values = colour.read()
    else:
        print("Going Right")
        colour_is_black = circle_obstacle(0, 'left', '<', False, colour_is_black, 0.5)
        motors.run(-config.line_speed, config.line_speed, 0.3)
        colour_is_black = circle_obstacle(0, 'straight', '>', False, colour_is_black, 0.5)
        motors.run(-config.line_speed, -config.line_speed, 0.3)

        print("Right Obstacle Loop Phase")
        while not colour_is_black[1]:
            colour_is_black = circle_obstacle(0, 'right', '>', True, colour_is_black, 1)
            # motors.run(-config.line_speed, -config.line_speed, 0.3)
            colour_is_black = circle_obstacle(0, 'straight', '>', True, colour_is_black, 1)
            motors.run(-config.line_speed, -config.line_speed, 0.3)

        motors.run(config.line_speed + 5, -config.line_speed - 5, 1.2)
        motors.run(config.line_speed, config.line_speed, 0.3)
        colour_values = colour.read()
        while colour_values[4] < 40 and turn_count < 500:
            turn_count += 1
            colour_values = colour.read()

def silver_check(colour_values, silver_count):
    global silver_min
    if (colour_values[0] > silver_min or colour_values[1] > silver_min or colour_values[3] > silver_min or colour_values[4] > silver_min) and colour_values[2] > 30:
        silver_count += 1
    
    return silver_count