import time
start_time = time.perf_counter()

import os
print(f"1: {time.perf_counter() - start_time:.2f}")
import sys
print(f"2: {time.perf_counter() - start_time:.2f}")

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)
print(f"3: {time.perf_counter() - start_time:.2f}")

import camera
print(f"4: {time.perf_counter() - start_time:.2f}")
import laser_sensors
print(f"5: {time.perf_counter() - start_time:.2f}")
import touch_sensors
print(f"6: {time.perf_counter() - start_time:.2f}")
import oled_display
print(f"7: {time.perf_counter() - start_time:.2f}")
import motors
print(f"8: {time.perf_counter() - start_time:.2f}")
import gyroscope
print(f"9: {time.perf_counter() - start_time:.2f}")
import led
print(f"10: {time.perf_counter() - start_time:.2f}")
import line
print(f"11: {time.perf_counter() - start_time:.2f}")
import config
print(f"12: {time.perf_counter() - start_time:.2f}")

def main() -> None:
    oled_display.initialise()
    print(f"13: {time.perf_counter() - start_time:.2f}")
    laser_sensors.initialise()
    print(f"14: {time.perf_counter() - start_time:.2f}")
    touch_sensors.initialise()
    print(f"15: {time.perf_counter() - start_time:.2f}")
    motors.initialise()
    print(f"16: {time.perf_counter() - start_time:.2f}")
    camera.initialise("line")
    print(f"17: {time.perf_counter() - start_time:.2f}")
    gyroscope.initialise()
    print(f"18: {time.perf_counter() - start_time:.2f}")
    
    motors.run(0, 0)
    oled_display.reset()
    led.off()
    
    config.update_log(["INITIALISATION", f"({time.perf_counter() - start_time:.2f})"], [24, 15])
    print()
    
    try:
        while True:
            line.main()

    finally:
        camera.close()
        oled_display.reset()
        motors.run(0, 0)
    
if __name__ == "__main__": main()