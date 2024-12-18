import time

import gpio
import laser_sensors
import touch_sensors
import oled_display
import camera

try:
    start_time = time.time()
    gpio.initialise()
    oled_display.initialise()
    laser_sensors.initialise()
    touch_sensors.initialise()
    camera.initialise()

    input(f"({time.time() - start_time:.2f}) Press enter to begin program! ")
    oled_display.reset()

    while True:
        oled_display.text("Hello World", 0, 0)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    gpio.cleanup()
    oled_display.reset()
