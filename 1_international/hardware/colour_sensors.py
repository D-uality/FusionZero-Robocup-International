from core.shared_imports import os, time, board, ADC, AnalogIn
from core.utilities import debug
from core.listener import listener

class ColourSensors():
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

            debug( ["INITIALISATION", "COLOUR SENSORS", ", ".join(list(map(str, self.__min_values))), ", ".join(list(map(str, self.__max_values)))], [25, 25, 23, 23] )
        
        else:
            debug( ["INITIALISATION", "COLOUR SENSORS", "x"], [25, 25, 50] )
            exit()
            
    def read_raw(self) -> list[int]:
        values = [int(AnalogIn(self.__ADC, channel).value / 256) for channel in range(0, 5)]
        return values
    
    def read(self) -> list[int]:
        mapped_values = []
        
        raw_values = self.read_raw()
        
        for i in range(5):
            mapped_value = (raw_values[i] - self.__min_values[i]) * 100 / (self.__max_values[i] - self.__min_values[i])
            mapped_values.append(int(mapped_value))
            
        return mapped_values
    
    def calibrate(self) -> None:
        # Rest calibration values
        self.__min_values = [255 for _ in range(0, 5)]
        self.__max_values = [0   for _ in range(0, 5)]
        
        start_time = time.perf_counter()
        
        while True:
            # Calibrate for 5 seconds
            if time.perf_counter() - start_time > 5: break
            
            analog_values = self.read_raw()

            # Update min, max for each sensor
            for i in range(5):
                self.__min_values[i] = min(self.__min_values[i], analog_values[i])
                self.__max_values[i] = max(self.__max_values[i], analog_values[i])
                
            debug( ["CALIBRATION", ", ".join(list(map(str, self.__min_values))), ", ".join(list(map(str, self.__max_values)))], [24, 30, 30] )
        
        with open(self.__CALIBRATION_FILE, "w") as file:
            file.write(",".join(map(str, self.__min_values)) + "\n")
            file.write(",".join(map(str, self.__max_values)) + "\n")
        
        listener.mode.value = 0