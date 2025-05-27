from core.shared_imports import time, ServoKit, ADC, board, AnalogIn
from core.utilities import debug

class Claw():
    def __init__(self):
        self.__i2c = board.I2C()
        self.__ADC = ADC.ADS7830(self.__i2c)
        
        self.__lifter_pin = 9
        self.__closer_pin = 8
        self.__pca = ServoKit(channels=16)
        
        self.__pca.servo[self.__lifter_pin].angle = 160
        self.__pca.servo[self.__closer_pin].angle = 90
        
        self.spaces = ["", ""]
        
        debug(["INITIALISATION", "CLAW", "✓"], [24, 14, 50])
    
    def lift(self, target_angle: int, time_delay: float = 0) -> None:
        if target_angle < 20: target_angle = 20
        if target_angle > 160: target_angle = 160
        current_angle = self.__pca.servo[self.__lifter_pin].angle

        if current_angle == target_angle: return None

        if current_angle > target_angle:
            while current_angle > target_angle:
                current_angle -= 1
                self.__pca.servo[self.__lifter_pin].angle = current_angle
                time.sleep(time_delay)

        elif current_angle < target_angle:
            while current_angle < target_angle:
                current_angle += 1
                self.__pca.servo[self.__lifter_pin].angle = current_angle
                time.sleep(time_delay)

        else:
            self.__pca.servo[self.__lifter_pin].angle = target_angle
            time.sleep(time_delay)
        
    def close(self, angle: int) -> None:
        self.__pca.servo[self.__closer_pin].angle = angle
    
    def read(self):
        values = [int(AnalogIn(self.__ADC, channel).value / 256) for channel in range(6, 8)]
        
        for i in range(0, len(self.spaces)):
            if   values[i] < 160: self.spaces[i] = ""
            elif values[i] < 230: self.spaces[i] = "dead"
            else:                 self.spaces[i] = "live"
        
        return self.spaces