import os
import time

import board
import adafruit_ads7830.ads7830 as ADC
from adafruit_ads7830.analog_in import AnalogIn

from main import debug

class cCOLOUR_SENSORS():
    def __init__(self):
        self.__i2c = board.I2C()
        self.__ADC = ADC.ADS7830(self.__i2c)

        self.__CURRENT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
        self.__CALIBRATION_FILE = os.path.join(self.__CURRENT_DIRECTORY, "calibration_values.txt")
        
        if os.path.exists(self.__CALIBRATION_FILE):
            with open(self.__CALIBRATION_FILE, "r") as file:
                lines = file.readlines()
                self.__min_values = list(map(int, lines[0].strip().split(",")))
                self.__max_values = list(map(int, lines[1].strip().split(",")))

            main.debug( ["INITIALISATION", "COLOUR SENSORS", ", ".join(list(map(str, self.__min_values))), ", ".join(list(map(str, self.__max_values)))], [24, 24, 30, 30] )
        
        else:
            main.debug( ["INITIALISATION", "COLOUR SENSORS", "x"], [24, 24, 50] )
            exit()
            
    def read_raw(self) -> list[int]:
        values = [int(AnalogIn(self.__ADC, channel).value / 256) for channel in range(0, 7)]
        return values
    
    def read(self) -> list[int]:
        mapped_values = []
        
        raw_values = self.read_raw()
        
        for i in range(7):
            mapped_value = (raw_values[i] - self.__min_values[i]) * 100 / (self.__max_values[i] - self.__min_values[i])
            mapped_values.append(int(mapped_value))
            
        return mapped_value
    
    def calibrate(self) -> None:
        # Rest calibration values
        self.__min_values = [255 for _ in range(0, 255)]
        self.__max_values = [0   for _ in range(0, 255)]
        
        start_time = time.perf_counter()
        
        while True:
            # Calibrate for 5 seconds
            if time.perf_counter - start_time > 5: break
            
            analog_values = self.read_raw()

            # Update min, max for each sensor
            for i in range(7):
                self.__min_values[i] = min(self.__min_values[i], analog_values[i])
                self.__max_values[i] = max(self.__max_values[i], analog_values[i])
                
            main.debug( ["CALIBRATION", ", ".join(list(map(str, self.__min_values))), ", ".join(list(map(str, self.__max_values)))], [24, 30, 30] )
        
        with open(self.__CALIBRATION_FILE, "w") as file:
            file.write(",".join(map(str, self.__min_values)) + "\n")
            file.write(",".join(map(str, self.__max_values)) + "\n")