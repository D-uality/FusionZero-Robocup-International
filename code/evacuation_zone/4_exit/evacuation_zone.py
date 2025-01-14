import time
import cv2
import numpy as np
from random import randint
import config
import camera
import motors
import touch_sensors
import laser_sensors
from typing import Optional

def find(search_function: callable) -> None:
    def search_while(v1: int, v2: int, time_constraint: float, search_function: callable, conditional_function: callable = None) -> Optional[int]:
        start_time = time.time()
        motors.run(v1, v2)
        
        initial_condition = conditional_function() if conditional_function else None

        while time.time() - start_time < time_constraint:
            print(config.update_log([f"SEARCH WHILE", f"{v1}", f"{v2}", f"{search_function.__name__}", f"{config.victim_count}", f"{time.time() - start_time:.2f}"], [24, 8, 8, 10, 6, 6]))
            image = camera.capture_array()

            if config.X11: cv2.imshow("image", image)

            x = search_function(image)

            if x is not None: return 1
            
            if conditional_function is not None:
                current_condition = conditional_function()
                if current_condition != initial_condition: return 0

        motors.run(0, 0)

        return None

    found_status = 0

    while found_status != 1:
        found_status = search_while(v1=config.evacuation_speed, v2=config.evacuation_speed, time_constraint=3.5, search_function=search_function, conditional_function=touch_sensors.read)

        if found_status == 1: continue
        elif found_status == 0: motors.run(-config.evacuation_speed, -config.evacuation_speed, 1.2)

        time_delay = 7 if found_status is None else randint(800, 1600) / 1000

        found_status = search_while(v1=config.evacuation_speed * 0.62, v2=-config.evacuation_speed * 0.62, search_function=search_function, time_constraint=time_delay)

    motors.run(0, 0)

def route(search_function: callable, kP: float) -> bool:
    distance = laser_sensors.read([config.x_shut_pins[1]])

    while distance[0] > config.approach_distance:
        image = camera.capture_array()
        distance = laser_sensors.read([config.x_shut_pins[1]])
        x = search_function(image)

        if x is None: return False

        scalar = 0.5 + (distance[0] - 15) * (0.5 / 85)
        error = config.WIDTH / 2 - x
        turn = int(error * kP)
        v1, v2 = scalar * (config.evacuation_speed-turn), scalar * (config.evacuation_speed+turn)
        motors.run(v1, v2)

        if config.X11: cv2.imshow("image", image)
        print(config.update_log([f"ROUTE", f"{error}", f"{scalar:.2f}", f"{turn}"], [24, 12, 12]))

    motors.run(0, 0, 0.5)
    motors.run_until(-config.evacuation_speed * 0.62, -config.evacuation_speed * 0.62, laser_sensors.read, 1, ">=", config.approach_distance, "ROUTE BACK")

    return True

def align(search_function: callable, step_time: float) -> bool:
    image = camera.capture_array()

    x = search_function(image)
    if x is None: return False

    error = config.WIDTH / 2 - x

    while error > 2:
        image = camera.capture_array()
        x = search_function(image)
        if x is None: return False
        
        error = config.WIDTH / 2 - x
        motors.run(-config.evacuation_speed * 0.62, config.evacuation_speed * 0.62, step_time)
        motors.run(0, 0, step_time)

        if config.X11: cv2.imshow("image", image)
        print(config.update_log([f"ALIGN RIGHT", f"{error}"], [24, 12]))

    motors.run(0, 0, 0.3)

    while error < -2:
        image = camera.capture_array()
        x = search_function(image)
        if x is None: return False

        error = config.WIDTH / 2 - x
        motors.run(config.evacuation_speed * 0.62, -config.evacuation_speed * 0.62, step_time)
        motors.run(0, 0, step_time)
    
        if config.X11: cv2.imshow("image", image)
        print(config.update_log([f"ALIGN RIGHT", f"{error}"], [24, 12]))

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

                if x < 0 or x >= config.WIDTH or y < 0 or y >= config.HEIGHT: continue
                points.append((x, y))

            return points
        
        y_levels = []

        for _ in range(trials):
            time.sleep(time_step)

            image = camera.capture_array()

            yellow_mask = cv2.inRange(image, (0, 30, 50), (30, 140, 200))
            contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours: continue

            largest_contour = max(contours, key=cv2.contourArea)
            _, y, _, h = cv2.boundingRect(largest_contour)

            if config.X11: 
                cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
                cv2.imshow("image", image)

            if y + h/2 > 135: y_levels.append(0)
            else:             y_levels.append(1)

        average = sum(y_levels) / trials

        black_count = 0
        points = generate_random_points(80, 25, 20) + generate_random_points(240, 25, 20)
        print(points)

        image = camera.capture_array()

        black_mask = cv2.inRange(image, (0, 0, 0), (40, 40, 40))
        
        for x, y in points:
            print(black_mask[y, x], end="   ")
            if black_mask[y, x] == 255: black_count += 1

        print(config.update_log([f"PRESENCE CHECK", f"{black_count}", f"{average:.2f}", f"{black_count > len(points) * 0.5}"], [24, 10, 10, 10]))
            
        if average > 0.5:
            if black_count < len(points) * 0.5 and config.victim_count < 2:    return True
            elif black_count > len(points) * 0.5 and config.victim_count == 2: return True
            else:                                                              return False
        else:                                                                  return False

    config.update_log(["GRAB", "CLAW DOWN"], [24, 24])
    motors.claw_step(0, 0.005)
    config.update_log(["GRAB", "MOVE FORWARDS"], [24, 24])
    motors.run(config.evacuation_speed * 0.8, config.evacuation_speed * 0.8, 1.3)
    config.update_log(["GRAB", "CLAW CLOSE"], [24, 24])
    motors.claw_step(90, 0.007)
    config.update_log(["GRAB", "MOVE BACKWARDS"], [24, 24])
    motors.run(-config.evacuation_speed * 0.8, -config.evacuation_speed * 0.8, 1)
    config.update_log(["GRAB", "CLAW READJUST"], [24, 24])
    motors.claw_step(75, 0.05)
    motors.claw_step(90, 0.05)
    config.update_log(["GRAB", "CLAW CHECK"], [24, 24])
    motors.claw_step(110, 0.005)

    time.sleep(0.3)
    return presence_check(25, 0.01)

def dump() -> None:
    motors.run_until(config.evacuation_speed, config.evacuation_speed, touch_sensors.read, 0, "==", 0, "FORWARDS")
    motors.run_until(config.evacuation_speed, config.evacuation_speed, touch_sensors.read, 1, "==", 0, "FORWARDS")

    motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.3)
    motors.claw_step(90, 0.005)
    
    time.sleep(1)

    motors.claw_step(270, 0)

    motors.run(config.evacuation_speed, -config.evacuation_speed, randint(300, 1600) / 1000)

def exit() -> None:
    touch_values = touch_sensors.read()
    laser_values = laser_sensors.read()

    if touch_values[0] == 0 or touch_values[1] == 0:
        motors.run(-config.evacuation_speed, -config.evacuation_speed, 0.5)
        motors.run(config.evacuation_speed * 1.3, config.evacuation_speed * 0.7, 0.8)

    elif laser_values[0] > 20:
        motors.run(config.evacuation_speed, config.evacuation_speed, 1.3)
        motors.run(-config.evacuation_speed, config.evacuation_speed, 1)
        motors.run_until(














