import time
import cv2
import numpy as np
from random import randint

from config import *
import laser_sensors
import touch_sensors
import oled_display
import camera
import motors
import live_victims

def search_while(v1, v2, time_constraint, conditional_function=None):
    start_time = time.time()
    motors.run(v1, v2)
    
    initial_condition = conditional_function() if conditional_function else None

    while time.time() - start_time < time_constraint:
        print(f"(SEARCH WHILE) ({v1}, {v2})   |    {time.time()-start_time:.2f}")
        image = camera.capture_array()

        live_x, _ = live_victims.find(image, 7)

        if live_x is not None: return 1
        
        if conditional_function:
            current_condition = conditional_function()
            if current_condition != initial_condition: return 0

        if X11: cv2.imshow("image", image)

    motors.run(0, 0)

    return None

def find_live(base_speed):
    found = 0

    while found != 1:
        found = search_while(v1=base_speed, v2=base_speed, time_constraint=3.5, conditional_function=touch_sensors.read)

        if found == 1: continue
        elif found == 0: motors.run(-base_speed, -base_speed, 1.2)

        time_delay = 3.5 if found is None else randint(800, 1600) / 1000
        
        found = search_while(v1=base_speed, v2=-base_speed, time_constraint=time_delay)

    motors.run(0, 0)

def grab(base_speed):
    print(f"(GRAB) claw down")
    motors.claw_step(0, 0.005)
    print(f"(GRAB) move forwards")
    motors.run(base_speed * 0.8, base_speed * 0.8, 1.3)
    print(f"(GRAB) claw close")
    motors.claw_step(90, 0.01)
    print(f"(GRAB) move backwards")
    motors.run(-base_speed * 0.8, -base_speed * 0.8, 1)
    print(f"(GRAB) claw open to readjust")
    motors.claw_step(75, 0.05)
    motors.claw_step(90, 0.05)
    print(f"(GRAB) claw check")
    motors.claw_step(110, 0.005)

    time.sleep(0.01)
    return presence_check(50)

def presence_check(trials):

    results = []

    for i in range(0, trials):
        print(f"(PRESENCE CHECK)")
        time.sleep(0.02)
        image = camera.capture_array()
        yellow = cv2.inRange(image, (0, 30, 50), (30, 140, 200))

        contours, _ = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    

        if not contours: 
            results.append(0)
            continue
        
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)

        if X11: cv2.imshow("image", image)

        if y + h/2 > 135:
            results.append(0)
            continue
        
        results.append(1)

    average = sum(results) / trials
    print(f"(PRESENCE CHECK) {average=}")
    if average > 0.5: return True
    else: return False

def dump(base_speed):
    motors.run_until(base_speed, base_speed, touch_sensors.read, 0, "==", 0, "FORWARDS")
    motors.run_until(base_speed, base_speed, touch_sensors.read, 1, "==", 0, "FORWARDS")

    motors.run(-base_speed, -base_speed, 0.3)
    motors.claw_step(90, 0.005)
    
    time.sleep(1)

    motors.claw_step(270, 0)

    motors.run(base_speed, -base_speed, randint(300, 600) / 1000)
