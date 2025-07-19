from core.shared_imports import GPIO, board, time, adafruit_vl53l1x
from core.utilities import debug
from hardware.motors import Motors

class LaserSensors():
    def __init__(self, motors: Motors):
        self.TIMING_BUDGET = 20
        
        self.__x_shut_pins = [23, 24, 25]
        self.__tof_sensors = []
        
        self.__fails = 0
        self.__last_values = [0, 0, 0]
        
        self.__motors = motors
        self.__setup()

    def __setup(self):
        for pin in self.__x_shut_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
            time.sleep(0.1)
            
        for pin_number, x_shut_pin in enumerate(self.__x_shut_pins):
            self.__change_address(pin_number, x_shut_pin)

        for sensor in self.__tof_sensors:
            sensor.start_ranging()
        
    def __change_address(self, pin_number: int, x_shut_pin: int) -> None:
        attempts = 10

        for attempt in range(1, attempts + 1):
            try:
                GPIO.output(x_shut_pin, GPIO.HIGH)
                time.sleep(0.2)

                sensor = adafruit_vl53l1x.VL53L1X(board.I2C())
                sensor.distance_mode = 1 
                sensor.timing_budget = self.TIMING_BUDGET

                self.__tof_sensors.append(sensor)

                if pin_number < len(self.__x_shut_pins) - 1:
                    sensor.set_address(pin_number + 0x30)

                debug(["INITIALISATION", f"LASERS {pin_number}", "✓"], [25, 25, 50])
                break

            except Exception as e:
                if attempt == attempts:
                    raise e
                else:
                    print(f"INITIALISATION: LASER {x_shut_pin} {e}")
                    time.sleep(0.1)

    def read(self, pins=None) -> list[int]:
        if self.__fails > 5:
            print("LASER SENSORS FAILED!")
            self.__motors.run(0, 0)
            
            self.__tof_sensors = []
            self.__setup()
            self.__fails = 0
        
        values = []
        if pins is None: sensors = self.__tof_sensors
        else:            sensors = [self.__tof_sensors[i] for i in pins]

        for sensor in sensors:
            try:
                distance = sensor.distance
                
                if distance is None: distance = 255
                values.append(distance)
                
                
            except Exception as e:
                print(f"Error reading sensor: {e}")
                values.append(None)

        
        # self.__fails = self.__fails + 1 if None in values and None in self.__last_values else 0
        self.__last_values = values
        
        return values