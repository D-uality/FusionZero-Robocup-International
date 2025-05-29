from core.shared_imports import GPIO, board, time, adafruit_vl53l1x, Optional
from core.utilities import debug

class LaserSensors():
    def __init__(self):
        # self.x_shut_pins = [24, 25]
        self.__x_shut_pins = [25, 24]
        self.__tof_sensors = []
        
        self.__fails = 0
        self.__last_values = [0, 0, 0]
        
        self.__setup()

    def __setup(self):
        for pin in self.__x_shut_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
            
        for pin_number, x_shut_pin in enumerate(self.__x_shut_pins):
            self.__change_address(pin_number, x_shut_pin)

        for sensor in self.__tof_sensors:
            sensor.start_ranging()
        
    def __change_address(self, pin_number: int, x_shut_pin: int) -> None:
        attempts = 5

        for attempt in range(1, attempts + 1):
            try:
                GPIO.output(x_shut_pin, GPIO.HIGH)
                time.sleep(0.1)

                sensor_i2c = adafruit_vl53l1x.VL53L1X(board.I2C())
                self.__tof_sensors.append(sensor_i2c)

                if pin_number < len(self.__x_shut_pins) - 1:
                    sensor_i2c.set_address(pin_number + 0x30)

                debug(["INITIALISATION", f"LASERS {pin_number}", "✓"], [25, 25, 50])
                break

            except Exception as e:
                if attempt == attempts:
                    raise e
                else:
                    print(f"INITIALISATION: LASER {x_shut_pin} {e}")
                    time.sleep(0.1)

    def read(self, pins=None) -> list[int]:
        print(self.__fails)
        if self.__fails > 5:
            
            print("LASER SENSORS FAILED!")
            self.__setup()
            
            self.__fails = 0
        
        values = []
        if pins is None: sensors = self.__tof_sensors
        else:            sensors = [self.__tof_sensors[i] for i in pins]

        for sensor in sensors:
            try:
                values.append(sensor.distance)
                
            except Exception as e:
                print(f"Error reading sensor: {e}")
                values.append(None)

        self.__fails += 1 if not all(values) and not all(self.__last_values) else 0
        self.__last_values = values
        
        return values