import os
import board
import adafruit_ads7830.ads7830 as ADC
from adafruit_ads7830.analog_in import AnalogIn
import config

class c_colour_sensors():
    def __init__(self):
        i2c = board.I2C()
        self.adc = ADC.ADS7830(i2c)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.CALIBRATION_FILE = os.path.join(current_dir, "calibration_values.txt")

        self.calibrated_min, self.calibrated_max = self.load_calibration_values()

    def load_calibration_values(self) -> tuple:
        if os.path.exists(self.CALIBRATION_FILE):
            with open(self.CALIBRATION_FILE, "r") as f:
                lines = f.readlines()
                min_values = list(map(int, lines[0].strip().split(",")))
                max_values = list(map(int, lines[1].strip().split(",")))

                config.update(["INITIALISATION", f"min: {min_values}", f"max: {max_values}"], [config.method_size, 16, 16])
        else:
            raise Exception("Calibration values not found!")

        return min_values, max_values
    
    def save_calibration_values(self) -> None:
        with open(self.CALIBRATION_FILE, "w") as f:
            f.write(",".join(map(str, self.calibrated_min)) + "\n")
            f.write(",".join(map(str, self.calibrated_max)) + "\n")
    
    def read_raw(self) -> list[int]:
        analog_readings = []
        for channel in range(7):
            analog_readings.append(int(AnalogIn(self.adc, channel).value / 256))
        
        return analog_readings

    def read(self) -> list[int]:
        raw_values = self.read_raw()
        calibrated_values = []

        for i, value in enumerate(raw_values):
            calibrated_values.append(int((value - self.calibrated_min[i]) / (self.calibrated_max[i] - self.calibrated_min[i]) * 100))

        return calibrated_values