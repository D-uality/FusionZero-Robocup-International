from core.shared_imports import time, ServoKit, ADC, board, AnalogIn
from core.utilities import debug, user_at_host

class Claw():
    def __init__(self):
        TRIALS = 50
        self.debug = False
        
        self.__i2c = board.I2C()
        self.__ADC = ADC.ADS7830(self.__i2c)
        
        self.lifter_pin = 9
        self.closer_pin = 8
        self.pca = ServoKit(channels=16)
        
        if user_at_host == "frederick@raspberrypi":
            self.EMPTY_TOLERANCE = 20
            self.OPPOSITE_LIVE_TOLERANCE = 20
            self.LIVE_TOLERANCE = 254
            
            self.left_cup = AnalogIn(self.__ADC, 0)
            self.right_cup = AnalogIn(self.__ADC, 1)
            
            self.lifter_angle = [20, 160]
            self.pca.servo[self.lifter_pin].angle = 160
            self.pca.servo[self.closer_pin].angle = 90
        
            """
            EMPTY:
            110 42
            
            DEAD:
            159 42
            110 55
            
            115 237
            237 110
            
            255 80
            150 230
            
            """
        
        else:
            self.EMPTY_TOLERANCE = 100
            self.OPPOSITE_LIVE_TOLERANCE = 100
            self.LIVE_TOLERANCE = 240
            
            self.left_cup = AnalogIn(self.__ADC, 0)
            self.right_cup = AnalogIn(self.__ADC, 1)
            
            self.lifter_angle = [30, 160]
            self.pca.servo[self.lifter_pin].angle = 160
            self.pca.servo[self.closer_pin].angle = 90

        self.analogs = [self.left_cup, self.right_cup]
        
        self.spaces = ["", ""]

        # Calibrate
        self.__empty_average = [0, 0]
        
        for _ in range(0, TRIALS):
            values = [int(self.analogs[i].value / 256) for i in range(0, len(self.analogs))]
            for i, value in enumerate(values): self.__empty_average[i] += value
            
            time.sleep(0.005)
        
        self.close(90)
        
        # Find average
        for i in range(0, len(self.__empty_average)):
            self.__empty_average[i] = self.__empty_average[i] / TRIALS
                    
        debug(["INITIALISATION", "CLAW", "✓"], [25, 25, 50])
    
    def lift(self, target_angle: int, time_delay: float = 0) -> None:
        if target_angle < self.lifter_angle[0]: target_angle = self.lifter_angle[0]
        if target_angle > self.lifter_angle[1]: target_angle = self.lifter_angle[1]
        current_angle = self.pca.servo[self.lifter_pin].angle

        if current_angle == target_angle: return None

        if current_angle > target_angle:
            while current_angle > target_angle:
                current_angle -= 1
                self.pca.servo[self.lifter_pin].angle = current_angle
                time.sleep(time_delay)

        elif current_angle < target_angle:
            while current_angle < target_angle:
                current_angle += 1
                self.pca.servo[self.lifter_pin].angle = current_angle
                time.sleep(time_delay)

        else:
            self.pca.servo[self.lifter_pin].angle = target_angle
            time.sleep(time_delay)
        
    def close(self, angle: int) -> None:
        self.pca.servo[self.closer_pin].angle = angle
    
    def read(self) -> list[int]:
        TRIALS = 15
        
        averages = [0, 0]
        
        try:
            # Find average reading
            for _ in range(0, TRIALS):
                values = [int(self.analogs[i].value / 256) for i in range(0, len(self.analogs))]
                for i, value in enumerate(values): averages[i] += value
                
                time.sleep(0.001)
            
            for i, average, in enumerate(averages):
                average = average / TRIALS
                if self.debug: print(f"{average:.2f}, {self.__empty_average[i]:.2f}", end="     ")
                # If opposite side has live
                tolerance =  self.OPPOSITE_LIVE_TOLERANCE if self.spaces[1 if i == 0 else 0] == "live" else self.EMPTY_TOLERANCE
                
                if average < self.LIVE_TOLERANCE:
                    self.spaces[i] = ""
                else:
                    self.spaces[i] = "live"
                # if average < self.__empty_average[i] + tolerance:
                #     # print(f"self.__empty_average[i]: {self.__empty_average[i]}, tolerance: {tolerance}, sum: {self.__empty_average[i] + tolerance}")
                #     self.spaces[i] = ""
                # elif average > self.LIVE_TOLERANCE:
                #     self.spaces[i] = "live"
                # else:
                #     self.spaces[i] = "dead"
                    
            return [averages[i] / TRIALS for i in range(0, len(averages))]
        
        except RuntimeError as e:
            debug( [f"ERROR", f"CLAW", f"{e}"], [30, 20, 50] )