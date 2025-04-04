import RPi.GPIO as GPIO
import adafruit_vl53l1x
import board
import time
import config
import oled_display

i2c = board.I2C()
tof_sensors = []

def initialise() -> None:
    def change_address(pin_number: int, x_shut_pin: int) -> None:
        # try:
            GPIO.output(x_shut_pin, GPIO.HIGH)
            sensor_i2c = adafruit_vl53l1x.VL53L1X(i2c)

            tof_sensors.append(sensor_i2c)
            if pin_number < len(config.x_shut_pins) - 1: sensor_i2c.set_address(pin_number + 0x29)
            
            print([f"ToF[{pin_number}]", "✓"])
            oled_display.text(f"ToF[{pin_number}]: ✓", 0, 0 + 10 * pin_number)
        # except Exception as e:
        #     print([f"ToF[{pin_number}]", "X", f"{e}"])
        #     oled_display.text(f"ToF[{pin_number}]: x", 0, 0 + 10 * pin_number)

    for x_shut_pin in config.x_shut_pins:
        GPIO.setup(x_shut_pin, GPIO.OUT)
        GPIO.output(x_shut_pin, GPIO.LOW)
        
        time.sleep(0.05)

    for pin_number, x_shut_pin in enumerate(config.x_shut_pins):
        change_address(pin_number, x_shut_pin)

    for sensor in tof_sensors:
        sensor.start_ranging()

    read()

def read(pins=config.x_shut_pins) -> list[int]:
    indices = [i for i, pin in enumerate(config.x_shut_pins) if pin in pins]
    print(indices, tof_sensors)
    sensors = [tof_sensors[i] for i in indices]

    values = []

    for sensor_number, sensor in enumerate(sensors):
        try:
            while not sensor.data_ready or sensor.distance is None:
                time.sleep(0.00001)

            values.append(sensor.distance)
            sensor.clear_interrupt()
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"Failed reading ToF[{sensor_number}]: {str(e)}")
            values.append(0)

    return values
