from core.shared_imports import GPIO, board, time, adafruit_vl53l1x, Optional
from core.utilities import debug

class LaserSensors():
    def __init__(self):
        # self.x_shut_pins = [23, 24, 25]  # Left, Middle, Right
        self.x_shut_pins = [23]  # Left, Right
        self.tof_sensors = []
        
        for pin in self.x_shut_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
            
        for pin_number, x_shut_pin in enumerate(self.x_shut_pins):
            self.change_address(pin_number, x_shut_pin)

        for sensor in self.tof_sensors:
            sensor.start_ranging()

    def change_address(self, pin_number: int, x_shut_pin: int) -> None:
        attempts = 5

        for attempt in range(1, attempts + 1):
            try:
                GPIO.output(x_shut_pin, GPIO.HIGH)
                time.sleep(0.1)

                sensor_i2c = adafruit_vl53l1x.VL53L1X(board.I2C())
                self.tof_sensors.append(sensor_i2c)

                if pin_number < len(self.x_shut_pins) - 1:
                    sensor_i2c.set_address(pin_number + 0x30)

                debug(["INITIALISATION", f"LASERS {pin_number}", "✓"], [24, 14, 50])
                break

            except Exception as e:
                if attempt == attempts:
                    raise e
                else:
                    print(f"INITIALISATION: LASER {x_shut_pin} {e}")
                    time.sleep(0.1)

    def read(self, pins=None) -> list[int]:
        values = []
        if pins is None:
            sensors = self.tof_sensors
        else:
            sensors = [self.tof_sensors[i] for i in pins]

        for sensor in sensors:
            try:
                values.append(sensor.distance)
            except Exception as e:
                print(f"Error reading sensor: {e}")
                values.append(None)
        return values