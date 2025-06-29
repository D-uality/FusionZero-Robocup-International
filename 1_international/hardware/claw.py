from numpy import empty
from core.shared_imports import time, ServoKit, ADC, board, AnalogIn, socket, getpass
from core.utilities import debug

class Claw():
    def __init__(self):
        TRIALS = 50
        self.debug = False
        
        self.__i2c = board.I2C()
        self.__ADC = ADC.ADS7830(self.__i2c)
        
        self.lifter_pin = 9
        self.closer_pin = 8
        self.pca = ServoKit(channels=16)
        
        self.a0 = AnalogIn(self.__ADC, 0)
        self.a1 = AnalogIn(self.__ADC, 1)
        self.analogs = [self.a0, self.a1]
        
        username = getpass.getuser()
        hostname = socket.gethostname()
        self.user_at_host = f"{username}@{hostname}"
        
        if self.user_at_host == "frederick@raspberrypi":
            self.pca.servo[self.lifter_pin].angle = 160
            self.pca.servo[self.closer_pin].angle = 90
        else:
            self.pca.servo[self.lifter_pin].angle = 105
            self.pca.servo[self.closer_pin].angle = 120
        
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
        if target_angle < 20: target_angle = 20
        if target_angle > 160: target_angle = 160
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
        EMPTY_TOLORANCE = 20
        OPPOSITE_LIVE_TOLORANCE = 20
        
        averages = [0, 0]
        
        try:
            # Find average reading
            for _ in range(0, TRIALS):
                values = [int(self.analogs[i].value / 256) for i in range(0, len(self.analogs))]
                for i, value in enumerate(values): averages[i] += value
                
                time.sleep(0.005)
            
            for i, average, in enumerate(averages):
                average = average / TRIALS
                if self.debug: print(f"{average:.2f}, {self.__empty_average[i]:.2f}", end="     ")
                # If opposite side has live
                tolorane =  OPPOSITE_LIVE_TOLORANCE if self.spaces[0 if i == 1 else 0] == "live" else EMPTY_TOLORANCE
                
                if self.__empty_average[i] - tolorane < average < self.__empty_average[i] + tolorane:
                    self.spaces[i] = ""
                elif average > 250:
                    self.spaces[i] = "live"
                else:
                    self.spaces[i] = "dead"
                    
            return [averages[i] / TRIALS for i in range(0, len(averages))]
        
        except RuntimeError as e:
            debug( [f"ERROR", f"CLAW", f"{e}"], [30, 20, 50] )