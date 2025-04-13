import RPi.GPIO as GPIO
import adafruit_vl53l1x
import board
import time
import config
import oled_display
import motors

i2c = board.I2C()
tof_sensors = []

def initialise() -> None:
    for x_shut_pin in config.x_shut_pins:
        GPIO.setup(x_shut_pin, GPIO.OUT)
        GPIO.output(x_shut_pin, GPIO.LOW)

    for pin_number, x_shut_pin in enumerate(config.x_shut_pins):
        change_address(pin_number, x_shut_pin)

    for sensor in tof_sensors:
        sensor.start_ranging()

    read()
    
def change_address(pin_number: int, x_shut_pin: int) -> None:
    attempts = 5
    
    for attempt in range(1, attempts + 1):
        try:
            GPIO.output(x_shut_pin, GPIO.HIGH)
            time.sleep(0.3)
            
            sensor_i2c = adafruit_vl53l1x.VL53L1X(i2c)
            tof_sensors.append(sensor_i2c)
            
            if pin_number < len(config.x_shut_pins) - 1:
                sensor_i2c.set_address(pin_number + 0x30)
            
            config.update_log(["INITIALISATION", f"LASER {pin_number}", "✓"], [24, 15, 50])
            print()
            oled_display.text(f"ToF[{pin_number}]: ✓", 0, 0 + 10 * pin_number)
            
            break
        
        except Exception as e:
            if attempt == attempts:
                raise e
            else:
                config.update_log(["INITIALISATION", f"LASER {x_shut_pin}", f"{e}"], [24, 15, 50])
                print()
                time.sleep(0.1)

def read(pins=config.x_shut_pins, display=False) -> list[int]:
    indices = [i for i, pin in enumerate(config.x_shut_pins) if pin in pins]
    sensors = [tof_sensors[i] for i in indices]

    values = []

    for sensor_number, sensor in enumerate(sensors):
        try:
            start_time = time.time()
            while not (sensor.data_ready or sensor.distance is None) and time.time() - start_time < 0.001:
                time.sleep(0.00001)

            values.append(sensor.distance if sensor.distance is not None else 1600)
            sensor.clear_interrupt()
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"Failed reading ToF[{sensor_number}]: {str(e)}")
            values.append(0)
            
            # Try re-initialsing all sensors
            motors.run(0, 0)
            initialise()            

    if display:
        print(f"L: {values}", end=", ")

    return values
