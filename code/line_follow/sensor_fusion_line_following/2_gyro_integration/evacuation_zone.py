import time
import cv2
import numpy as np
from random import randint

from listener import listener
import config
import camera
import motors
import touch_sensors
import laser_sensors
import led
import colour

from typing import Optional
import victims
import triangles

silver_min = 110
black_max = 20

def main():
    motors.run(0, 0)
    camera.close()
    camera.initialise("evac")
    
    entrance_exit_align("silver")
    
    motors.run(0, 0)
    # Let camera warm up
    for _ in range(0, 3):
        image = camera.capture_array()
        # if config.X11: cv2.imshow("image", image)

    motors.run(0, 0)
                
    start_time = time.time()
    
    while True:
        if config.victim_count == 3 or time.time() - start_time > 5 * 60: break

        search_type = victims.live if config.victim_count < 2 else victims.dead
        motors.claw_step(270, 0.00001)

        # TODO Fix sequencing
        # TODO find -> route -> alignment -> grab [FAIL]
        # TODO MUST return to route again, instead of back into alignemnet

        find(search_function=search_type)
        
        route_success = route(search_function=search_type, kP=0.35)
        
        if not route_success:
            config.update_log(["ROUTE FAILED"], [24])            
            continue
            
        align_success = align(search_function=search_type, step_time=0.01)
        
        if not align_success:
            config.update_log(["ALIGN FAILED"], [24])
            
            # Warmup camera again
            for _ in range(0, 3):
                image = camera.capture_array()
                motors.run(0, 0)
            continue
        
        grab_success = grab()
        
        if not grab_success:
            config.update_log(["GRAB FAILED"], [24])
            motors.run(-config.evacuation_speed, -config.evacuation_speed, 1)
            continue

        motors.claw_step(230, 0.005)
        triangles.find()
        dump()
        
        config.victim_count += 1

    exit_evacuation_zone()
    
    entrance_exit_align("black")
    
    motors.run(0, 0)
    camera.close()
    camera.initialise("line")
    motors.run(0, 0)

    listener.mode = 1

def entrance_exit_align(align_type: str) -> None:
    if align_type == "silver":   

        motors.run( config.evacuation_speed * 0.7,  config.evacuation_speed * 0.7, 0.7)
        motors.run(-config.evacuation_speed * 0.3, -config.evacuation_speed * 0.3)
        left_silver, right_silver = None, None
        
        while left_silver is None and right_silver is None:
            colour_values = colour.read()

            if colour_values[0] > silver_min or colour_values[1] > silver_min:
                left_silver = True
                print("Left")
            elif colour_values[3] > silver_min or colour_values[4] > silver_min:
                right_silver = True
                print("Right")
        
        motors.run(config.evacuation_speed * 0.5, config.evacuation_speed * 0.5, 0.4)

        if(left_silver): 
            motors.run_until(7, -config.evacuation_speed * 0.5, colour.read, 4, ">=", silver_min, "Right")
            
        for i in range(4):
            forwards_speed = config.evacuation_speed * 0.5
            other_speed = 8
            
            motors.run      ( 0, 0, 0.2)
            motors.run_until(-forwards_speed,     other_speed, colour.read, 0, ">=", silver_min, "LEFT SILVER")
            
            motors.run      ( 0, 0, 0.2)
            motors.run_until(    other_speed, -forwards_speed, colour.read, 4, ">=", silver_min, "RIGHT SILVER")
            
            motors.run      (0, 0, 0.2)
            motors.run_until( forwards_speed,     other_speed, colour.read, 0, "<=", silver_min, "LEFT WHITE")
            
            motors.run      ( 0, 0, 0.2)
            motors.run_until(    other_speed,  forwards_speed, colour.read, 4, "<=", silver_min, "RIGHT WHITE")
                    
    elif align_type == "black":
        motors.run(-config.evacuation_speed*0.7, -config.evacuation_speed*0.7, 0.7)
        motors.run(config.evacuation_speed * 0.5, config.evacuation_speed * 0.5)
        left_black, right_black = None, None
        while left_black is None and right_black is None:
            colour_values = colour.read()

            if colour_values[0] < 40 or colour_values[1] < 40:
                left_black = True
                print("Left")
            elif colour_values[3] < 40 or colour_values[4] < 40:
                right_black = True
                print("Right")

        if(left_black): 
            motors.run_until(-5, config.evacuation_speed * 0.5, colour.read, 4, "<=", 30, "Right")
            
        for i in range(4):
            motors.run(0, 0, 0.2)
            motors.run_until(-config.evacuation_speed * 0.5, -5, colour.read, 0, ">=", 60, "Left")
            motors.run(0, 0, 0.2)
            motors.run_until(-5, -config.evacuation_speed * 0.5, colour.read, 4, ">=", 60, "Right")
            motors.run(0, 0, 0.2)
            motors.run_until(config.evacuation_speed * 0.5, -5, colour.read, 0, "<=", 30, "Left")
            motors.run(0, 0, 0.2)
            motors.run_until(-5, config.evacuation_speed * 0.5, colour.read, 4, "<=", 30, "Right")

        for i in range(0, int(config.evacuation_speed * 0.7)):
            motors.run(-config.evacuation_speed*0.7+i, -config.evacuation_speed*0.7+i, 0.03)

def find(search_function: callable) -> None:
    def search_while(v1: int, v2: int, time_constraint: float, search_function: callable, conditional_function: callable = None) -> Optional[int]:
        start_time = time.time()
        motors.run(v1, v2)
        
        initial_condition = conditional_function() if conditional_function else None
        exit_entrance_count = 0

        while time.time() - start_time < time_constraint:
            image = camera.capture_array()
            x = search_function(image)
            
            # If victim in image
            if x is not None: return 1
            
            # Check conditional function and whether it has updated or not
            if conditional_function is not None:
                current_condition = conditional_function()
                if current_condition != initial_condition: return 0

            # Ensure we stay within the evacuation_zone
            colour_values = colour.read()
            exit_entrance_values = [1 if value >= 110 or value <= 30 else 0 for value in colour_values]
            exit_entrance_count = exit_entrance_count + sum(exit_entrance_values) if sum(exit_entrance_values) >= 1 else 0

            if exit_entrance_count >= 5:
                config.update_log([f"EXIT_ENTRANCE FOUND"], [24])
                print()
                return 0

            # Display debug
            if config.X11: cv2.imshow("image", image)
            config.update_log([f"SEARCH WHILE", f"{v1} {v2}", f"{search_function.__name__}", f"victim_count: {config.victim_count}", ",".join(list(map(str, colour_values))), f"ee_count: {exit_entrance_count}", f"{time.time() - start_time:.2f}"], [24, 10, 10, 15, 30, 15, 6])
            print()
            
        motors.run(0, 0)
        return None

    found_status = 0

    while found_status != 1:
        # Search while moving forwards
        found_status = search_while(v1=config.evacuation_speed, v2=config.evacuation_speed, time_constraint=3.5, search_function=search_function, conditional_function=touch_sensors.read)

        if found_status == 1:
            # If vicim found, exit loop
            continue
        elif found_status == 0:
            # Else move backwards as conditional_function was triggered
            motors.run(-config.evacuation_speed, -config.evacuation_speed, 1.2)

        # Differentiate between wall and blank space
        time_delay = 7 if found_status is None else randint(800, 1600) / 1000

        found_status = search_while(v1=config.evacuation_speed * 0.65, v2=-config.evacuation_speed * 0.65, search_function=search_function, time_constraint=time_delay)

    motors.run(0, 0)

def route(search_function: callable, kP: float) -> bool:
    distance = laser_sensors.read([config.x_shut_pins[1]])
    exit_entrance_count = 0
    
    config.update_log(["ROUTE (INITIAL)", f"{distance[0]:.2f}"], [24, 10])
    print()

    while distance[0] > config.approach_distance and distance[0] != 0:
        image = camera.capture_array()
        distance = laser_sensors.read([config.x_shut_pins[1]])
        x = search_function(image)

        if x is None: return False

        scalar = 0.5 + (distance[0] - 15) * (0.5 / 85)
        error = config.EVACUATION_WIDTH / 2 - x
        turn = int(error * kP)
        v1, v2 = scalar * (config.evacuation_speed-turn), scalar * (config.evacuation_speed+turn)
        motors.run(v1, v2)
        
        # Ensure we stay within the evacuation_zone
        colour_values = colour.read()
        exit_entrance_values = [1 if value >= 110 or value <= 30 else 0 for value in colour_values]
        exit_entrance_count = exit_entrance_count + sum(exit_entrance_values) if sum(exit_entrance_values) >= 1 else 0

        if exit_entrance_count >= 5:
            config.update_log([f"EXIT_ENTRANCE FOUND"], [24])
            print()
            return False

        if config.X11: cv2.imshow("image", image)
        config.update_log([f"ROUTE", f"{error}", f"{scalar:.2f}", f"{turn}", f"{exit_entrance_count}"], [24, 12, 12, 12])
        print()

    motors.run(0, 0, 0.5)
    motors.run_until(-config.evacuation_speed * 0.62, -config.evacuation_speed * 0.62, laser_sensors.read, 1, ">=", config.approach_distance, "ROUTE BACK")

    return True

def align(search_function: callable, step_time: float) -> bool:
    image = camera.capture_array()

    x = search_function(image)
    if x is None: return False

    error = config.EVACUATION_WIDTH / 2 - x

    while error > 2:
        image = camera.capture_array()
        x = search_function(image)
        if x is None: return False
        
        error = config.EVACUATION_WIDTH / 2 - x
        motors.run(-config.evacuation_speed, config.evacuation_speed, step_time)
        motors.run(0, 0, step_time)

        if config.X11: cv2.imshow("image", image)
        config.update_log([f"ALIGN LEFT", f"{error}"], [24, 12])
        print()

    motors.run(0, 0, 0.3)

    while error < -2:
        image = camera.capture_array()
        x = search_function(image)
        if x is None: return False

        error = config.EVACUATION_WIDTH / 2 - x
        motors.run(config.evacuation_speed, -config.evacuation_speed, step_time)
        motors.run(0, 0, step_time)
    
        if config.X11: cv2.imshow("image", image)
        config.update_log([f"ALIGN RIGHT", f"{error}"], [24, 12])
        print()

    motors.run_until(-config.evacuation_speed * 0.62, -config.evacuation_speed * 0.62, laser_sensors.read, 1, ">=", config.approach_distance, "ALIGN BACK")
    
    # motors.run(0, 0, 0.3)
    # motors.run_until( config.evacuation_speed * 0.62,  config.evacuation_speed * 0.62, laser_sensors.read, 1, "<=", config.approach_distance, "ALIGN FRONT")

    return True

def grab() -> bool:
    def presence_check(trials: int, time_step: float) -> bool:
        def generate_random_points(x_centre: int, y_centre: int, radius: int) -> list[tuple[int, int]]:
            cycle_count = 0
            points = []

            while cycle_count < 30:
                angle = randint(0, 360)
                x = x_centre + int(radius * np.cos(angle))
                y = y_centre + int(radius * np.sin(angle))

                cycle_count += 1

                if x < 0 or x >= config.EVACUATION_WIDTH or y < 0 or y >= config.EVACUATION_HEIGHT: continue
                points.append((x, y))

            return points
        
        y_levels = []

        for _ in range(0, trials):
            time.sleep(time_step)

            image = camera.capture_array()
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            blue_mask = cv2.inRange(hsv_image, (100, 0, 0), (140, 255, 60))
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours: continue

            largest_contour = max(contours, key=cv2.contourArea)
            _, y, _, h = cv2.boundingRect(largest_contour)

            if config.X11: 
                cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
                cv2.imshow("image", image)

            config.update_log(["PRESENCE CHECK", f"{y + h/2}"], [24, 10])
            print()

            if y + h/2 > 75: y_levels.append(0)
            else:            y_levels.append(1)

        average = sum(y_levels) / trials

        black_count = 0
        cv2.circle(image, (120, 16), 16, (255, 0, 0), 1)
        cv2.circle(image, (config.EVACUATION_WIDTH - 120, 16), 16, (255, 0, 0), 1)
        
        if config.X11: cv2.imshow("image", image)
        
        points = generate_random_points(120, 16, 16) + generate_random_points(config.EVACUATION_WIDTH - 120, 16, 16)

        image = camera.capture_array()
        black_mask = cv2.inRange(image, (0, 0, 0), (40, 40, 40))
        
        for x, y in points: 
            if black_mask[y, x] == 255: black_count += 1

        success_fail = True
        
        if average > 0.5:
            if black_count < len(points) * 0.5 and config.victim_count < 2:    success_fail = True
            elif black_count > len(points) * 0.5 and config.victim_count == 2: success_fail = True
            else:                                                              success_fail = False
        else:                                                                  success_fail = False
        
        config.update_log([f"PRESENCE CHECK", f"{black_count}", f"{average:.2f}", f"{black_count > len(points) * 0.5}", f"{success_fail=}"], [24, 10, 10, 10, 20])
        print()    
        
        return success_fail

    config.update_log(["GRAB", "CLAW DOWN"], [24, 24])
    print()
    motors.claw_step(0, 0.005)
    
    config.update_log(["GRAB", "MOVE FORWARDS"], [24, 24])
    print()
    motors.run(config.evacuation_speed, config.evacuation_speed, 0.7)
    motors.run(0, 0)
    
    config.update_log(["GRAB", "CLAW CLOSE"], [24, 24])
    print()
    motors.claw_step(120, 0.004)
    time.sleep(1)
    
    config.update_log(["GRAB", "MOVE BACKWARDS"], [24, 24])
    print()
    motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.7)
    motors.run(0, 0)
    
    config.update_log(["GRAB", "CLAW READJUST"], [24, 24])
    print()
    motors.claw_step(100, 0.03)
    motors.claw_step(120, 0.03)
    
    config.update_log(["GRAB", "CLAW CHECK"], [24, 24])
    print()
    motors.claw_step(150, 0.001)

    success_fail = presence_check(25, 0)
    config.update_log(["GRAB", f"{success_fail}"], [24, 20])
    print()
    
    return success_fail

def dump() -> None:
    motors.run_until(config.evacuation_speed, config.evacuation_speed, touch_sensors.read, 0, "==", 0, "FORWARDS LEFT")
    motors.run_until(config.evacuation_speed, config.evacuation_speed, touch_sensors.read, 1, "==", 0, "FORWARDS RIGHT")

    motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.3)
    motors.run(0, 0)
    motors.claw_step(90, 0.005)
    
    time.sleep(1)
    motors.claw_step(270, 0.001)

    motors.run(config.evacuation_speed, -config.evacuation_speed, randint(300, 1600) / 1000)
    motors.run(0, 0)

# def exit_evacuation_zone() -> bool:
#     global black_count
#     black_count = 0

#     laser_values = laser_sensors.read()
#     touch_values = touch_sensors.read()

#     print(config.update_log(["EXITING", f"{touch_values}", f"{laser_values}"], [24, 10, 6]))

#     def validate_exit() -> bool:
#         global black_count
#         colour_values = colour.read()

#         for value in colour_values[:5]:
#             if value <= 20:  
#                 black_count += 1
#                 return True
#             elif value >= 130: 
#                 return False
#             else:
#                 black_count = 0

#         return None

#     # Forwards until wall or line reached
#     motors.run(config.evacuation_speed, config.evacuation_speed)
#     while True:
#         touch_values = touch_sensors.read([config.touch_pins[0], config.touch_pins[1]])
#         laser_values = laser_sensors.read()

#         if (laser_values[1] < 5 and laser_values[1] is not None and laser_values[1] != 0) or touch_values[0] == 0 or touch_values[1] == 0:
#             print("Initial Wall Touched")
#             break
#         elif validate_exit() is not None:
#             print("Line Found")
#             break

#     # Wall until sees black line
#     while black_count < 1:
#         print(f"Black: {black_count}")
#         touch_values = touch_sensors.read([config.touch_pins[0], config.touch_pins[1]])
#         laser_values = laser_sensors.read()

#         # Wall Touched
#         if (laser_values[1] < 5 and laser_values[1] is not None and laser_values[1] != 0) or touch_values[0] == 0 or touch_values[1] == 0:
#             print("Wall Touched")
#             motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.5)
#             motors.run(config.evacuation_speed, -config.evacuation_speed, 0.6)
#             motors.run(config.evacuation_speed, config.evacuation_speed, 0.6)

#         # Line Found
#         if validate_exit() is True:
#             print("BLACK LINE")
#         elif validate_exit() is False:
#             print("SILVER LINE")
#             motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.6)
#             motors.run(config.evacuation_speed, -config.evacuation_speed, 0.7)
#             motors.run(config.evacuation_speed, config.evacuation_speed, 0.9)

#         # Wall Follow
#         kP = 5
#         wall_distance = 10
#         if laser_values[0] != 0:
#             print(laser_values[0])
#             turn = max(min(kP * (wall_distance - laser_values[0]), config.evacuation_speed*1.3), -config.evacuation_speed*1.3)
#             motors.run(config.evacuation_speed+turn, config.evacuation_speed-turn)
    
#     entry_exit_align("black")

#     motors.run(0, 0)
#     camera.close()
#     camera.initialise("line")

def exit_evacuation_zone() -> None:
    # Move towards a wall
    black_count = silver_count = 0
    
    while True:
        touch_values = touch_sensors.read()
        colour_values = colour.read()
        
        config.update_log(["EXITING", "MOVING TOWARDS WALL", ",".join(list(map(str, touch_values))), ",".join(list(map(str, colour_values)))], [24, 30, 10, 30])
        print()
        motors.run(config.evacuation_speed * 1.5, config.evacuation_speed * 1.5)
        
        silver_count, black_count = validate_exit(colour_values, black_count, silver_count)
        
        if black_count >= 5:
            config.update_log(["EXITING", "FOUND EXIT!"], [24, 30])
            print()
            return True
        
        elif silver_count >= 5:
            config.update_log(["EXITING", "FOUND SILVER"], [24, 30])
            print()
            
            motors.run(0, 0, 0.3)
            entrance_exit_align("silver")
            motors.run(-config.evacuation_speed, -config.evacuation_speed, 1)
            motors.run(0, 0, 0.3)
            motors.run(22, -22, 2.9)
            motors.run(0, 0, 0.3)
            motors.run_until( 22      ,  22      , laser_sensors.read, 0, "<=", 20, "MOVING TILL WALL")
            break
        
        elif sum(touch_values) != 2:
            config.update_log(["EXITING", "FOUND WALL"], [24, 30])
            print()
            motors.run      (-config.evacuation_speed, -config.evacuation_speed, 0.5)
            motors.run_until( config.evacuation_speed, -config.evacuation_speed, laser_sensors.read, 0, "<=", 20, "MOVING TILL WALL")
            motors.run      ( config.evacuation_speed, -config.evacuation_speed, 0.8)
            break
        
    # Follow wall till exit is found
    gap_initial = False
    
    while True:
        touch_values = touch_sensors.read()
        colour_values = colour.read()
        laser_value = laser_sensors.read([config.x_shut_pins[0]])[0]
        
        config.update_log(["EXITING", "FINDING GAPS", ",".join(list(map(str, touch_values))), ",".join(list(map(str, colour_values))), f"Laser: {laser_value}", f"Silver: {silver_count}", f"Black: {black_count}"], [24, 30, 10, 30, 10, 6, 6])
        print()
        motors.run(config.evacuation_speed * 0.8, config.evacuation_speed * 1.1)
        
        silver_count, black_count = validate_exit(colour_values, black_count, silver_count)
        
        if laser_value is not None:
            if laser_value > 30 and not gap_initial:
                # Turn to face gap
                motors.run(0, 0, 0.3)
                motors.run(22, 22, 1.3)
                motors.run(-22, 22, 2.9)
                gap_initial = True
        
        if black_count >= 5:
            config.update_log(["EXITING", "FOUND EXIT!"], [24, 30])
            print()
            break
        
        elif silver_count >= 5:
            config.update_log(["EXITING", "FOUND SILVER"], [24, 30])
            print()
            
            motors.run(0, 0, 0.3)
            entrance_exit_align("silver")
            motors.run(-config.evacuation_speed, -config.evacuation_speed, 1.2)
            motors.run(0, 0, 0.3)
            motors.run(22, -22, 2.9)
            motors.run(0, 0, 0.3)
            motors.run_until( 22,  22, laser_sensors.read, 0, "<=", 20, "MOVING TILL WALL")
            
        elif sum(touch_values) != 2:
            config.update_log(["EXITING", "FOUND WALL"], [24, 30])
            print()
            motors.run      (-config.evacuation_speed, -config.evacuation_speed, 0.35)
            motors.run      ( config.evacuation_speed, -config.evacuation_speed, 0.35)

# def exit_evacuation_zone() -> None:
#     # Move towards a wall
#     black_count = silver_count = 0

#     motors.run(config.evacuation_speed * 0.7, config.evacuation_speed * 0.7)
#     while True:
#         colour_values = colour.read()
#         touch_values = touch_sensors.read([config.touch_pins[0], config.touch_pins[1]])
#         laser_values = laser_sensors.read()

#         silver_count, black_count = validate_exit(colour_values, black_count,silver_count)

#         if (laser_values[1] < 5 and laser_values[1] != 0) or touch_values[0] == 0 or touch_values[1] == 0:
#             print("Initial Wall Touched")
#             break
#         elif silver_count != 0 or black_count != 0:
#             print("Line Found") 
#             break
        
#     # Follow wall till exit is found
#     while True:
#         touch_values = touch_sensors.read()
#         colour_values = colour.read()
#         laser_values = laser_sensors.read()
        
#         config.update_log(["EXITING", "FINDING GAPS", ",".join(list(map(str, touch_values))), ",".join(list(map(str, colour_values))), f"Laser: {laser_values}", f"Silver: {silver_count}", f"Black: {black_count}"], [24, 30, 10, 30, 10, 6, 6])
#         print()
#         motors.run(config.evacuation_speed * 0.5, config.evacuation_speed * 0.8)
        
#         silver_count, black_count = validate_exit(colour_values, black_count, silver_count)
        
#         if laser_values[0] is not None:
#             if laser_values[0] > 30:
#                 # Turn to face gap
#                 motors.run(0, 0, 0.3)
#                 motors.run(22, 22, 1.3)
#                 motors.run(-22, 22, 2.9)
        
#         if black_count >= 5:
#             config.update_log(["EXITING", "FOUND EXIT!"], [24, 30])
#             print()
#             break
        
#         elif silver_count >= 5:
#             config.update_log(["EXITING", "FOUND SILVER"], [24, 30])
#             print()
            
#             motors.run(0, 0, 0.3)
#             entrance_exit_align("silver")
#             motors.run(-config.evacuation_speed*0.7, -config.evacuation_speed*0.7, 1.6)
#             motors.run(0, 0, 0.3)
#             motors.run(22, -22, 2.9)
#             motors.run(0, 0, 0.3)
#             motors.run(22, 22, 1)
#             motors.run(0, 0, 1)
            
#         elif sum(touch_values) != 2 or (laser_values[1] < 5 and laser_values[1] is not None):
#             config.update_log(["EXITING", "FOUND WALL"], [24, 30])
#             print()
#             motors.run      (-config.evacuation_speed*0.7, -config.evacuation_speed*0.7, 0.5)
#             motors.run      ( config.evacuation_speed*0.7, -config.evacuation_speed*0.7, 0.5)
#             config.update_log(["EXIT FOUND!"], [24]) 
    
#     print()
    
#     entrance_exit_align("black")
    
#     motors.run(0, 0)

#     camera.close()
#     camera.initialise("line")

            
def validate_exit(colour_values: list[int], black_count: int, silver_count: int) -> bool:
    silver_values = [1 if value >= silver_min else 0 for value in colour_values[:5]]
    black_values  = [1 if value <= black_max  else 0 for value in colour_values[:5]]
    
    silver_count = silver_count + sum(silver_values) if sum(silver_values) >= 1 else 0
    black_count  = black_count  + sum(black_values)  if sum(black_values)  >= 1 else 0
    
    return silver_count, black_count

if __name__ == "__main__": main()
