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
import victims
import triangles

try:
    gpio.initialise()
    oled_display.initialise()
    laser_sensors.initialise()
    touch_sensors.initialise()
    camera.initialise()
    motors.initialise()

    input(f"({time.time() - start_time:.2f}) Press enter to begin program! ")
    oled_display.reset()

    while True:
        search_type = victims.live if victim_count < 2 else victims.dead
        motors.claw_step(270, 0)

        evacuation_zone.find(search_function=search_type)

        if evacuation_zone.route(search_function=search_type, kP=0.08):
            if evacuation_zone.align(search_function=search_type, step_time=0.01):
                if evacuation_zone.grab():
                    motors.claw_step(180, 0.005)
                    triangles.find()
                    evacuation_zone.dump()
                    victim_count += 1
                else:
                    motors.claw_step(0, 0)
                    motors.run(-base_evacuation_speed, -base_evacuation_speed, 1.2)
            else: motors.run(-base_evacuation_speed, -base_evacuation_speed, 1.2)
        else: motors.run(-base_evacuation_speed, base_evacuation_speed, 0.8)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    gpio.cleanup()
    oled_display.reset()
    camera.close()
    motors.run(0, 0)
    motors.claw_step(270, 0)