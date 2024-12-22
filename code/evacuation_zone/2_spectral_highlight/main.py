import time
start_time = time.time()
import cv2
import numpy as np
from random import randint

from config import *
import gpio
import laser_sensors
import touch_sensors
import oled_display
import camera
import motors
import live_victims

try:
    gpio.initialise()
    oled_display.initialise()
    laser_sensors.initialise()
    touch_sensors.initialise()
    camera.initialise()
    motors.initialise()

    input(f"({time.time() - start_time:.2f}) Press enter to begin program! ")
    oled_display.reset()

    display_found = False
    display_reset = False
    base_speed = 30
    target_distance = 20

    while True:
        start_time = time.time()
        image = camera.capture_array()

        motors.run(base_speed, base_speed)

        touch_values = touch_sensors.read([touch_pins[0], touch_pins[1]])
        if touch_values[0] == 0 or touch_values[1] == 0:
            motors.run(-base_speed, -base_speed, 1.2)

            spin_time, start_time = randint(800, 1600) / 1000, time.time()
            while time.time() - start_time < spin_time:
                motors.run(base_speed, -base_speed)

                image = camera.capture_array()
                live_x, _ = live_victims.find(image, 7)

                if live_x is not None: break
        
        live_x, _ = live_victims.find(image, 7)

        if live_x is not None and live_y is not None:
            if not display_found:
                oled_display.text("FOUND", 0, 0, size=20)
                display_found = True
                display_reset = False

            success = live_victims.route(v=base_speed, kP=0.08, target_distance=target_distance)

            if success:
                motors.claw_step(0, 0.01)
                motors.run(base_speed * 0.8, base_speed * 0.8, 1.6)
                motors.claw_step(90, 0.01)
                motors.run(-base_speed * 0.8, -base_speed * 0.8, 1)
                motors.claw_step(180, 0.01)
                motors.run(-base_speed, -base_speed, 2)

                input()

            motors.claw_step(270, 0.01)

        else:
            if not display_reset:
                oled_display.reset()
                display_found = False
                display_reset = True

        if X11:
            cv2.imshow("image", image)

        FPS = (1)/(time.time() - start_time)
        print(f"{FPS=:.2f}")

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    gpio.cleanup()
    oled_display.reset()
    camera.close()
    motors.run(0, 0)
    motors.claw_step(270, 0)