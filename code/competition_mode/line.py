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
red_threshold = [ [60, 80], [45, 65] ]
red_count = 0

touch_count = 0
laser_close_count = 0
last_uphill = 0

uphill_trigger = downhill_trigger = seasaw_trigger = evac_trigger = False
tilt_left_trigger = tilt_right_trigger = False
gap_trigger = False
evac_exited = False
camera_enable = False

def main(evacuation_zone_enable: bool = False) -> None:
    global main_loop_count, laser_close_count, silver_count, red_count, evac_trigger, evac_exited, touch_count

    green_signal = ""
    colour_values = colour.read()
    gyroscope_values = gyroscope.read()
    touch_values = touch_sensors.read()
    laser_value = laser_sensors.read([config.x_shut_pins[1]])

    if laser_value[0] is not None: laser_close_count = laser_close_count + 1 if laser_value[0] < 8 and laser_value[0] != 0 else 0
    touch_count = touch_count + 1 if sum(touch_values) != 2 else 0

    seasaw_check()
    silver_count = silver_check(colour_values, silver_count)

    if not camera_enable:
        gap_check(colour_values)
        green_signal = green_check(colour_values)
        if evac_exited: red_count = red_check(colour_values, red_count)
        # red_count = red_check(colour_values, red_count)
        
    if silver_count > 20:
        silver_count = 0

        evacuation_zone.main()
        main_loop_count = 0

        led.on()
        motors.run(0, 0, 2)

        evac_trigger = True
                    
    elif red_count > 15:
        red_count = 0
        print("Red Found")
        motors.run(0, 0, 10)

    elif touch_count > 150:
        avoid_obstacle()
    
    elif len(green_signal) > 0:
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
        uphill_trigger   = True if gyroscope_values[0] >=  15 else False
        downhill_trigger = True if gyroscope_values[0] <= -15 else False
        
    # tilt detection
    # if gyroscope_values[1] is not None:
    #     tilt_left_trigger  = True if gyroscope_values[1] >=  15 else False
    #     tilt_right_trigger = True if gyroscope_values[1] <= -15 else False

    # RESET DETECTION
    if colour_values[2] <= 40 and abs(camera_last_error) < 15 and abs(camera_integral) == 0:  
        if gap_trigger    and main_loop_count > 100: gap_trigger    = False
        if seasaw_trigger and main_loop_count > 150: seasaw_trigger = False
        
    if evac_trigger:
        if main_loop_count > 75: evac_trigger = False

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
    if   uphill_trigger:   kP, kI, kD, v = 0.2 , 0     ,  0  , 25
    elif downhill_trigger: kP, kI, kD, v = 0.5 , 0     ,  0  , 10
    elif gap_trigger:      kP, kI, kD, v = 1   , 0     ,  0  , config.line_speed
    elif seasaw_trigger:   kP, kI, kD, v = 1   , 0     ,  0  , config.line_speed
    elif evac_trigger:     kP, kI, kD, v = 0.5 , 0     ,  0  , 18
    else:                  kP, kI, kD, v = 1.45, 0     ,  0  , config.line_speed

    # Input method
    if uphill_trigger or downhill_trigger or gap_trigger or seasaw_trigger or evac_trigger:
        modifiers += " CAMERA"
        camera_enable = True
        led.on()

        image = camera.capture_array()
        transformed_image = camera.perspective_transform(image, modifiers)
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
        config.update_log([modifiers+" PID", f"{main_loop_count}", f"{angle:.2f} {colour_values[2]}", f"{error:.2f} {camera_integral:.2f} {camera_derivative:.2f}", f"{v1:.2f} {v2:.2f}", f"{silver_count} {laser_close_count} {touch_count} {red_count}"], [24, 8, 30, 30, 10, 30])
        
    else:
        camera_enable = False
        led.off()
        
        updated_colour_values = [min(value, 100) for value in colour_values]

        outer_error = 1 * (updated_colour_values[0] - updated_colour_values[4])
        inner_error = 1 * (updated_colour_values[1] - updated_colour_values[3])
        error = outer_error + inner_error
                
        integral_reset_count = integral_reset_count + 1 if abs(error) <= 15 and updated_colour_values[1] + updated_colour_values[3] >= 75 else 0
        
        if integral_reset_count >= min_integral_reset_count: ir_integral = 0
        elif ir_integral <= -50000: ir_integral = -49999
        elif ir_integral >=  50000: ir_integral =  49999
        else: ir_integral += error * 0.8 if abs(error) > 120 else error * 0.3
        ir_derivative = error - ir_last_error

        if updated_colour_values[0] > 70 and updated_colour_values[1] > 70 and updated_colour_values[3] > 70 and updated_colour_values[4] > 70:
            middle_multi = 0.2
        else:
            middle_multi = max(1 + (updated_colour_values[2] - 60)/100, 0)

        turn = middle_multi * (error * kP + ir_integral * kI + ir_derivative * kD)
        v1, v2 = v + turn, v - turn

        if   tilt_left_trigger:  v1, v2 = v1 + 5, v2 - 5
        elif tilt_right_trigger: v1, v2 = v1 - 5, v2 + 5
    
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
    left_black = colour_values[0] < 30 and colour_values[1] < 30

    # Right Mid
    right_black = colour_values[3] < 30 and colour_values[4] < 30
    # Double
    left_double_black  = left_black  and (colour_values[3] < 45 or colour_values[4] < 60)
    right_double_black = right_black and (colour_values[0] < 45 or colour_values[1] < 60)

    # Green
    

    if left_black and right_black and colour_values[2] > 40 and colour_values[5] + colour_values[6] >= 120:
        if colour_values[0] + colour_values[1] < colour_values[3] + colour_values[4]:
            signal = "fake left"
        else:
            signal = "fake right"
    elif colour_values[2] < 80 and abs(ir_integral) <= 1000:
        if (left_double_black or right_double_black) and ((colour_values[5] < 30 and colour_values[6] < 40) or (colour_values[5] < 40 and colour_values[6] < 30)):
            signal = "double"
            
        elif (left_black or left_double_black) and colour_values[5] < 30:
            signal = "left"
        
        elif (right_black or right_double_black) and colour_values[6] < 30:
            signal = "right"
        
    if len(signal) != 0: config.update_log(["GREEN CHECK", ",".join(list(map(str, colour_values))), signal], [24, 30, 10])

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
        motors.run(config.line_speed * 1, config.line_speed * 1, 0.3)
        motors.run_until(-config.line_speed, config.line_speed * 1.1, colour.read, 2, ">=", 40, "FIRST ALIGN")
        motors.run(0, 0, 0.15)
        motors.run(-config.line_speed, config.line_speed, 0.2)
        motors.run(0, 0, 0.15)
        motors.run_until(-config.line_speed, config.line_speed, colour.read, 2, "<=", 30, "SECOND ALIGN")
        motors.run(0, 0, 0.15)
        motors.run(-config.line_speed, config.line_speed, 0.4)
        motors.run(0, 0, 0.15)
        motors.run(config.line_speed, config.line_speed, 0.3)
        motors.run(0, 0, 0.15)

    elif signal == "right":
        motors.run(config.line_speed * 1, config.line_speed * 1, 0.3)
        motors.run_until(config.line_speed * 1.1, -config.line_speed, colour.read, 2, ">=", 40, "FIRST ALIGN")
        motors.run(0, 0, 0.15)
        motors.run(config.line_speed, -config.line_speed, 0.2)
        motors.run(0, 0, 0.15)
        motors.run_until(config.line_speed, -config.line_speed, colour.read, 2, "<=", 30, "SECOND ALIGN")
        motors.run(0, 0, 0.15)
        motors.run(config.line_speed, -config.line_speed, 0.4)
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
    global main_loop_count, gap_loop_count, gap_trigger

    white_values = [1 if value > 80 else 0 for value in colour_values]
    gap_loop_count = gap_loop_count + 1 if sum(white_values) == 7 else 0

    gap_trigger = True if gap_loop_count >= 40 else False
    
    if gap_trigger:
        main_loop_count = 0
        motors.run(-config.line_speed * 1.5, -config.line_speed * 1.5, 1)
        motors.run(0, 0, 0.3)
        
        image = camera.capture_array()
        if config.X11: cv2.imshow("image", image)

def seasaw_check():
    global downhill_trigger, last_uphill, seasaw_trigger
    
    if downhill_trigger and last_uphill < 40:
        last_uphill = 100
        main_loop_count = 0
        motors.run(0, 0, 2)
        motors.run(-config.line_speed, -config.line_speed, 1.5)
        seasaw_trigger = True

def avoid_obstacle() -> None:
    side_values = laser_sensors.read([config.x_shut_pins[0], config.x_shut_pins[2]])

    config.update_log(["OBSTACLE", "FINDING SIDE", ", ".join(list(map(str, side_values)))], [24, 50, 14])
    print()

    # Clockwise if left > right
    direction = "cw" if side_values[0] > side_values[1] else "ccw"

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
    for i in range(20):
        motors.run_until(1.2 * v1, 1.2*v2, laser_sensors.read, laser_pin, "<=", 20, "TURNING TILL OBSTACLE")
        motors.run(1.2 * v1, 1.2 * v2, 0.01)

    for i in range(35):
        motors.run_until(1.2 * v1, 1.2*v2, laser_sensors.read, laser_pin, ">=", 20, "TURNING PAST OBSTACLE")
        motors.run(1.2 * v1, 1.2 * v2, 0.01)

    motors.run(1.2 * v1, 1.2*v2, 0.5)

    motors.run(0, 0, 1)

    # Turn back onto obstacle
    motors.run_until(-v1, -v2, laser_sensors.read, laser_pin, "<=", 15, "TURNING TILL OBSTACLE")
    motors.run(-1.2 * v1, -1.2 * v2, 1.8)

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
    
    circle_obstacle(config.line_speed, config.line_speed, laser_pin, colour_black_pin, "<=", 13, "FORWARDS TILL OBSTACLE")
    motors.run(config.line_speed, config.line_speed, 0.5)

    while True:
        if circle_obstacle(config.line_speed, config.line_speed, laser_pin, colour_black_pin, ">=", 20, "FORWARDS TILL NOT OBSTACLE"): pass
        elif not initial_sequence: break
        motors.run(-config.line_speed, -config.line_speed, 0.4)
        motors.run(0, 0, 0.15)

        
        if circle_obstacle(v1, v2, laser_pin, colour_black_pin, "<=", 15, "TURNING TILL OBSTACLE"): pass
        elif not initial_sequence: break
        motors.run(0, 0, 0.15)

        if circle_obstacle(v1, v2, laser_pin, colour_black_pin, ">=", 18, "TURNING TILL NOT OBSTACLE"): pass
        elif not initial_sequence: break
        motors.run(v1, v2, 0.4)
        motors.run(0, 0, 0.15)

        if circle_obstacle(config.line_speed, config.line_speed, laser_pin, colour_black_pin, "<=", 18, "FORWARDS TILL OBSTACLE"): pass
        elif not initial_sequence: break
        motors.run(0, 0, 0.15)

        initial_sequence = False

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

def red_check(colour_values, red_count):
    global red_threshold, main_loop_count
    
    if main_loop_count < 50: return red_count

    if (red_threshold[0][0] < colour_values[5] < red_threshold[0][1] or red_threshold[1][0] < colour_values[6] < red_threshold[1][1]) and all(colour_values[i] > 70 and colour_values[i] < 120 for i in range(5)):
        red_count += 1
    else: 
        red_count = 0
    
    return red_count