from core.shared_imports import GPIO, board, time, adafruit_vl53l1x, Optional, mp
from core.utilities import debug
from hardware.motors import Motors

class LaserSensors():
    def __init__(self, motors: Motors):
        self.__x_shut_pins = [23, 24, 25]
        self.__tof_sensors = []
        self.__fails = 0
        self.__last_values = [0, 0, 0]
        self.__motors = motors
        
        self.__data_queue = mp.Queue()
        self.__exit_event = mp.Event()
        self.__reading_process = mp.Process(target=self.__background_read)
        
        self.__setup()
        self.start()

    def start(self):
        self.__reading_process.start()
    
    def stop(self):
        self.__exit_event.set()
        if self.__reading_process.is_alive():
            self.__reading_process.terminate()
            self.__reading_process.join()

    def read(self, pins=None) -> list[int]:
        try:
            values = self.__data_queue.get_nowait()
            if pins is not None:
                values = [values[i] for i in pins]
                
            self.__fails = self.__fails + 1 if None in values and None in self.__last_values else 0
            
            if self.__fails > 5:
                print("LASER SENSORS FAILED!")
                self.__setup()
                self.__motors.run(0, 0)
                self.__fails = 0
                
            self.__last_values = values
            return values
            
        except Exception:
            return self.__last_values

    def __background_read(self):
        try:
            while not self.__exit_event.is_set():
                values = []
                
                for sensor in self.__tof_sensors:
                    try:   values.append(sensor.distance)
                    except Exception as e:
                        print(f"Error reading sensor: {e}")
                        values.append(None)
                
                self.__data_queue.put(values)
                
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            pass

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
                sensor.timing_budget = 100

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