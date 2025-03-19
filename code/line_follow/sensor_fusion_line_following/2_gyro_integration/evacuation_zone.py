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

def main():
    camera.close()
    camera.initialise(config.EVACUATION_WIDTH, config.EVACUATION_HEIGHT)
    motors.run(0, 0, 2)
    
    start_time = time.time()
    motors.run(config.evacuation_speed, config.evacuation_speed, 0.5)
    motors.run(0, 0)
    
    while True:
        if config.victim_count == 3 or time.time() - start_time > 5 * 60: break

        search_type = victims.live if config.victim_count < 2 else victims.dead
        motors.claw_step(270, 0)

        find(search_function=search_type)

        if route(search_function=search_type, kP=0.35):
            if align(search_function=search_type, step_time=0.01):
                if grab():
                    motors.claw_step(210, 0.005)
                    triangles.find()
                    dump()
                    config.victim_count += 1
                else:
                    motors.claw_step(0, 0)
                    motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.8)
            else: motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.8)
            motors.run(0, 0)

    exit_evacuation_zone()

def find(search_function: callable) -> None:
    def search_while(v1: int, v2: int, time_constraint: float, search_function: callable, conditional_function: callable = None) -> Optional[int]:
        start_time = time.time()
        motors.run(v1, v2)
        
        initial_condition = conditional_function() if conditional_function else None
        exit_entrance_count = 0

        while time.time() - start_time < time_constraint:
            config.update_log([f"SEARCH WHILE", f"{v1}", f"{v2}", f"{search_function.__name__}", f"{config.victim_count}", f"{exit_entrance_count}", f"{time.time() - start_time:.2f}"], [24, 8, 8, 10, 6, 6, 6])
            print()
            image = camera.capture_array()
            
            if config.X11: cv2.imshow("image", image)

            x = search_function(image)

            if x is not None: return 1
            
            if conditional_function is not None:
                current_condition = conditional_function()
                if current_condition != initial_condition: return 0

                colour_values = colour.read()
                exit_entrance_values = [1 if value >= 120 or value <= 30 else 0 for value in colour_values]
                exit_entrance_count += 1 if sum(exit_entrance_values) >= 1 else 0

                if exit_entrance_count >= 10:
                    motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.8)
                    motors.run( config.evacuation_speed, -config.evacuation_speed, 2)
                    return 0
                else:
                    exit_entrance_count = 0

        motors.run(0, 0)

        return None

    found_status = 0

    while found_status != 1:
        found_status = search_while(v1=config.evacuation_speed, v2=config.evacuation_speed, time_constraint=3.5, search_function=search_function, conditional_function=touch_sensors.read)

        if found_status == 1: continue
        elif found_status == 0: motors.run(-config.evacuation_speed, -config.evacuation_speed, 1.2)
        motors.run(0, 0)

        time_delay = 7 if found_status is None else randint(800, 1600) / 1000

        found_status = search_while(v1=config.evacuation_speed * 0.62, v2=-config.evacuation_speed * 0.62, search_function=search_function, time_constraint=time_delay)

    motors.run(0, 0)

def route(search_function: callable, kP: float) -> bool:
    distance = laser_sensors.read([config.x_shut_pins[1]])
    exit_entrance_count = 0

    while distance[0] > config.approach_distance:
        image = camera.capture_array()
        distance = laser_sensors.read([config.x_shut_pins[1]])
        x = search_function(image)

        if x is None: return False

        scalar = 0.5 + (distance[0] - 15) * (0.5 / 85)
        error = config.EVACUATION_WIDTH / 2 - x
        turn = int(error * kP)
        v1, v2 = scalar * (config.evacuation_speed-turn), scalar * (config.evacuation_speed+turn)
        motors.run(v1, v2)
        
        colour_values = colour.read()
        exit_entrance_values = [1 if value >= 120 or value <= 30 else 0 for value in colour_values]
        exit_entrance_count += 1 if sum(exit_entrance_values) >= 1 else 0

        if exit_entrance_count >= 10:
            motors.run(-config,evacuation_speed, -config.evacuation_speed, 0.8)
            motors.run( config.evacuation_speed, -config.evacuation_speed, 2)
            return False
        else:
            exit_entrance_count = 0

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
        motors.run(config.evacuation_speed * 0.62, -config.evacuation_speed * 0.62, step_time)
        motors.run(0, 0, step_time)
    
        if config.X11: cv2.imshow("image", image)
        config.update_log([f"ALIGN RIGHT", f"{error}"], [24, 12])
        print()

    motors.run(0, 0, 0.3)
    motors.run_until(-config.evacuation_speed * 0.62, -config.evacuation_speed * 0.62, laser_sensors.read, 1, ">=", config.approach_distance, "ALIGN BACK")
    
    motors.run(0, 0, 0.3)
    motors.run_until( config.evacuation_speed * 0.62,  config.evacuation_speed * 0.62, laser_sensors.read, 1, "<=", config.approach_distance, "ALIGN FRONT")

    return True

def grab() -> bool:
    def presence_check(trials: int, time_step: float) -> Optional[bool]:
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

        for _ in range(trials):
            time.sleep(time_step)

            image = camera.capture_array()
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            blue_mask = cv2.inRange(hsv_image, (100, 0, 0), (160, 255, 60))
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours: continue

            largest_contour = max(contours, key=cv2.contourArea)
            _, y, _, h = cv2.boundingRect(largest_contour)

            if config.X11: 
                cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
                cv2.imshow("image", image)

            if y + h/2 > 144: y_levels.append(0)
            else:             y_levels.append(1)

        average = sum(y_levels) / trials

        black_count = 0
        points = generate_random_points(80, 25, 20) + generate_random_points(240, 25, 20)

        image = camera.capture_array()
        black_mask = cv2.inRange(image, (0, 0, 0), (40, 40, 40))
        
        for x, y in points: 
            if black_mask[y, x] == 255: black_count += 1

        config.update_log([f"PRESENCE CHECK", f"{black_count}", f"{average:.2f}", f"{black_count > len(points) * 0.5}"], [24, 10, 10, 10])
        print()    
        
        if average > 0.5:
            if black_count < len(points) * 0.5 and config.victim_count < 2:    return True
            elif black_count > len(points) * 0.5 and config.victim_count == 2: return True
            else:                                                              return False
        else:                                                                  return False

    config.update_log(["GRAB", "CLAW DOWN"], [24, 24])
    print()
    motors.claw_step(0, 0.005)
    config.update_log(["GRAB", "MOVE FORWARDS"], [24, 24])
    print()
    motors.run(config.evacuation_speed * 0.8, config.evacuation_speed * 0.8, 0.85)
    motors.run(0, 0)
    config.update_log(["GRAB", "CLAW CLOSE"], [24, 24])
    print()
    motors.claw_step(110, 0.004)
    time.sleep(1)
    config.update_log(["GRAB", "MOVE BACKWARDS"], [24, 24])
    print()
    motors.run(-config.evacuation_speed * 0.8, -config.evacuation_speed * 0.8, 1)
    motors.run(0, 0)
    config.update_log(["GRAB", "CLAW READJUST"], [24, 24])
    print()
    motors.claw_step( 85, 0.05)
    motors.claw_step(110, 0.05)
    config.update_log(["GRAB", "CLAW CHECK"], [24, 24])
    print()
    motors.claw_step(140, 0.001)

    time.sleep(0.3)
    return presence_check(25, 0.01)

def dump() -> None:
    motors.run_until(config.evacuation_speed, config.evacuation_speed, touch_sensors.read, 0, "==", 0, "FORWARDS")
    motors.run_until(config.evacuation_speed, config.evacuation_speed, touch_sensors.read, 1, "==", 0, "FORWARDS")

    motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.3)
    motors.run(0, 0)
    motors.claw_step(90, 0.005)
    
    time.sleep(1)

    motors.claw_step(270, 0)

    motors.run(config.evacuation_speed, -config.evacuation_speed, randint(300, 1600) / 1000)
    motors.run(0, 0)

def exit_evacuation_zone() -> bool:
    def validate_exit() -> bool:
        black_count, silver_count = 0, 0

        while True:
            motors.run(config.evacuation_speed, config.evacuation_speed)
            colour_values = colour.read()

            config.update_log(["APPROACHING", f"{colour_values}", f"{black_count}", f"{silver_count}"], [24, 24, 6, 6])
            print()

            for value in colour_values:
                if   value <= 20:  black_count  += 1
                elif value >= 135: silver_count += 1

            if    black_count > 10: return True
            elif silver_count > 10: return False
    
    motors.run_until(config.evacuation_speed, config.evacuation_speed, touch_sensors.read, 0, "==", 0)

    while True:
        touch_values = touch_sensors.read([config.touch_pins[0], config.touch_pins[1]])
        laser_values = laser_sensors.read([config.x_shut_pins[0]])

        config.update_log(["EXITING", f"{touch_values}", f"{laser_values}"], [24, 10, 6])
        print()

        if touch_values[0] == 0 or touch_values[1] == 0:
            motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.2)
            motors.run(config.evacuation_speed, -config.evacuation_speed, 0.3)
        elif laser_values[0] > 30:
            motors.run(config.evacuation_speed, config.evacuation_speed, 1.3)
            motors.run(-config.evacuation_speed, config.evacuation_speed, 1.3)

            if validate_exit():
                print("BLACK")
                return True
            else:
                print("SILVER")
                motors.run(-config.evacuation_speed, -config.evacuation_speed, 1.3)
                motors.run( config.evacuation_speed, -config.evacuation_speed, 1.3)
                motors.run( config.evacuation_speed,  config.evacuation_speed, 2)
        else:
            motors.run(config.evacuation_speed * 0.7, config.evacuation_speed)
            
if __name__ == "__main__": main()