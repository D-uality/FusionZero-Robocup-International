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
import evacuation_zone
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
    target_distance = 18
    base_speed = 30

    while True:
        motors.claw_step(270, 0)
        evacuation_zone.find_live(base_speed=base_speed)

        if live_victims.route(v=base_speed, kP=0.08, target_distance=target_distance):
            if live_victims.align(v=base_speed, target_distance=target_distance):
                evacuation_zone.grab(base_speed=base_speed)

                input()

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    gpio.cleanup()
    oled_display.reset()
    camera.close()
    motors.run(0, 0)
    motors.claw_step(270, 0)