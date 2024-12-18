from config import *
import RPi.GPIO as GPIO
import adafruit_vl53l1x
import board
import time

import oled_display

i2c = board.I2C()
tof_sensors = []

def initalise():
    def change_address(pin_number, x_shut_pin):
        try:
            print(f"\tSetting GPIO output for x_shut_pin {x_shut_pin} to HIGH")
            GPIO.output(x_shut_pin, GPIO.HIGH)
            print(f"\tInitializing VL53L1X sensor on I2C bus for pin {x_shut_pin}")
            sensor_i2c = adafruit_vl53l1x.VL53L1X(i2c)

            tof_sensors.append(sensor_i2c)

            if pin_number < len(x_shut_pins) - 1:
                print(f"\tSetting new I2C address for sensor: 0x{pin_number + 0x30:02X}")
                sensor_i2c.set_address(pin_number + 0x30)

            print(f"\t\tSuccess!")
            oled_display.text(f"ToF[{pin_number}]: ✓", 0, 0 + 10 * pin_number)
        except:
            print(f"\tToF[{pin_number}] failed to initalise, on pin {x_shut_pin}")
            oled_display.text(f"ToF[{pin_number}]: x", 0, 0 + 10 * pin_number)

    for x_shut_pin in x_shut_pins:
        GPIO.setup(x_shut_pin, GPIO.OUT)
        GPIO.output(x_shut_pin, GPIO.LOW)

    for pin_number, x_shut_pin in enumerate(x_shut_pins):
        print(f"Changing address for ToF[{pin_number}] at pin {x_shut_pin}")
        change_address(pin_number, x_shut_pin)

    for sensor in tof_sensors:
        sensor.start_ranging()

def read(pins=x_shut_pins):
    indices = [i for i, pin in enumerate(x_shut_pins) if pin in pins]
    sensors = [tof_sensors[i] for i in indices]

    values = []

    for sensor_number, sensor in enumerate(sensors):
        try:
            while not sensor.data_ready:
                time.sleep(0.00001)

            values.append(sensor.distance)
            sensor.clear_interrupt()
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"Failed reading ToF {sensor_number}: {str(e)}")
            values.append(0)

    print("Lasers:", values, end="    ")
    return values
