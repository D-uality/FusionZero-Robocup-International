from core.shared_imports import os, time, board, ADC, AnalogIn, socket, getpass
from core.utilities import debug
from core.listener import listener

class SilverSensor():
    def __init__(self):
        self.__i2c = board.I2C()
        self.__ADC = ADC.ADS7830(self.__i2c)
        username = getpass.getuser()
        hostname = socket.gethostname()
        self.user_at_host = f"{username}@{hostname}"
        
        if self.user_at_host == "frederick@raspberrypi":
            self.sensor_pin = 5
        else:
           self.sensor_pin = 5

        self.__CURRENT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
        self.__CALIBRATION_FILE = os.path.join(self.__CURRENT_DIRECTORY, "silver_values.txt")
        
        if os.path.exists(self.__CALIBRATION_FILE):
            with open(self.__CALIBRATION_FILE, "r") as file:
                lines = file.readlines()
                self.__min_value = int(lines[0])
                self.__max_value = int(lines[1])

            debug( ["INITIALISATION", "SILVER SENSOR", f"{self.__min_value}", f", {self.__max_value}"], [25, 25, 23, 23] )
        
        else:
            debug( ["INITIALISATION", "SILVER SENSOR", "x"], [25, 25, 50] )
            exit()
            
    def read_raw(self) -> list[int]:
        value = int(AnalogIn(self.__ADC, self.sensor_pin).value / 256)
        return value
    
    def read(self) -> list[int]:
        return int((self.read_raw() - self.__min_value) * 100 / (self.__max_value - self.__min_value))
    
        
    def calibrate(self) -> None:
        # Rest calibration values
        self.__min_value = 255
        self.__max_value = 0
        
        start_time = time.perf_counter()
        
        while True:
            # Calibrate for 5 seconds
            if time.perf_counter() - start_time > 10: break
            
            analog_value = self.read_raw()

            # Update min, max for each sensor
            self.__min_value = min(self.__min_value, analog_value)
            self.__max_value = max(self.__max_value, analog_value)
                
            debug( ["CALIBRATION, ", f"{self.__min_value}", f", {self.__max_value}"], [24, 30, 30] )
        
        with open(self.__CALIBRATION_FILE, "w") as file:
            file.write(f"{self.__min_value}" + "\n")
            file.write(f"{self.__max_value}" + "\n")
        
        print("Saved Silver Calibration Values!")
        
        listener.mode.value = 0