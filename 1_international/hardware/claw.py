from numpy import empty
from core.shared_imports import time, ServoKit, ADC, board, AnalogIn
from core.utilities import debug

class Claw():
    def __init__(self):
        TRIALS = 50
        
        self.__i2c = board.I2C()
        self.__ADC = ADC.ADS7830(self.__i2c)
        
        self.__lifter_pin = 9
        self.__closer_pin = 8
        self.__pca = ServoKit(channels=16)
        
        self.__pca.servo[self.__lifter_pin].angle = 160
        self.__pca.servo[self.__closer_pin].angle = 90
        
        self.spaces = ["", ""]

        # Calibrate
        self.__empty_average = [0, 0]
        
        for _ in range(0, TRIALS):
            values = [int(AnalogIn(self.__ADC, channel).value / 256) for channel in range(6, 8)]
            for i, value in enumerate(values): self.__empty_average[i] += value
            
            time.sleep(0.01)
        
        # Find average
        for i in range(0, len(self.__empty_average)):
            self.__empty_average[i] = self.__empty_average[i] / TRIALS
                    
        debug(["INITIALISATION", "CLAW", "✓"], [25, 25, 50])
    
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
        TRIALS = 15
        EMPTY_TOLORANCE = 5
        OPPOSITE_LIVE_TOLORANCE = 5
        
        averages = [0, 0]
        
        try:
            # Find average reading
            for _ in range(0, TRIALS):
                values = [int(AnalogIn(self.__ADC, channel).value / 256) for channel in range(6, 8)]
                for i, value in enumerate(values): averages[i] += value
                
                time.sleep(0.005)
            
            for i, average, in enumerate(averages):
                average = average / TRIALS
                
                # If opposite side has live
                tolorane =  OPPOSITE_LIVE_TOLORANCE if self.spaces[0 if i == 1 else 0] == "live" else EMPTY_TOLORANCE
                
                if self.__empty_average[i] - tolorane < average < self.__empty_average[i] + tolorane:
                    self.spaces[i] = ""
                elif average > 250:
                    self.spaces[i] = "live"
                else:
                    self.spaces[i] = "dead"
        
        except RuntimeError as e:
            debug( [f"ERROR", f"CLAW", f"{e}"], [30, 20, 50] )