from core.shared_imports import time, ServoKit, operator, socket, getpass
from core.utilities import debug

pca = ServoKit(channels=16)

class Motors():
    def __init__(self) -> None:
        self.servo_pins = [14, 13, 12, 10]
    
        username = getpass.getuser()
        hostname = socket.gethostname()
        user_at_host = f"{username}@{hostname}"
        
        if user_at_host == "frederick@raspberrypi":
            self.stop_angles = [88, 89, 88, 89]
            self.negative_gradients = [0.70106, 0.70440, 0.66426, 0.69425]
            self.negative_intercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
            self.positive_gradients = [0.70924, 0.63035, 0.67858, 0.69916]
            self.positive_intercepts = [4.1509, 6.5548, 3.5948, 4.2287]
            
        elif user_at_host == "aidan@raspberrypi":
            self.stop_angles = [97, 96, 96, 97]
            self.negative_gradients = [1.17106, 1.15876, 0.50274, 0.53452]
            self.negative_intercepts = [-3.8658, -1.5444, -2.8559, -2.4646]
            self.positive_gradients = [0.87580, 0.82601, 1.78133, 1.81726]
            self.positive_intercepts = [4.6227, 4.3606, 3.9117, 3.4639]
            
        else:
            print(user_at_host)
            raise ValueError(f"Unknown hostname: {hostname}")
       
        pca.servo[self.servo_pins[0]].angle = self.stop_angles[0]
        pca.servo[self.servo_pins[1]].angle = self.stop_angles[1]
        pca.servo[self.servo_pins[2]].angle = self.stop_angles[2]
        pca.servo[self.servo_pins[3]].angle = self.stop_angles[3]

        debug(["INITIALISATION", "MOTORS", "✓"], [24, 14, 50])
    
    def run(self, v1: float, v2: float, delay: float = 0) -> None:
        calculated_angles = [0, 0, 0, 0]

        for i in range(0, 2):
            if   (v1 < self.negative_intercepts[i]): calculated_angles[i] = self.negative_gradients[i] * v1 + self.negative_intercepts[i]
            elif (v1 > self.positive_intercepts[i]): calculated_angles[i] = self.positive_gradients[i] * v1 + self.positive_intercepts[i]
            else:                                    calculated_angles[i] = 0

            calculated_angles[i] = max(min(calculated_angles[i], 90), -90)

        v2 = v2 * -1

        for i in range(2, 4):
            if   (v2 < self.negative_intercepts[i]): calculated_angles[i] = self.negative_gradients[i] * v2 + self.negative_intercepts[i]
            elif (v2 > self.positive_intercepts[i]): calculated_angles[i] = self.positive_gradients[i] * v2 + self.positive_intercepts[i]
            else:                                   calculated_angles[i] = 0

            calculated_angles[i] = max(min(calculated_angles[i], 90), -90)
        try:
            pca.servo[self.servo_pins[0]].angle = max(min(self.stop_angles[0] + calculated_angles[0], 90+60), 90-40)
            pca.servo[self.servo_pins[1]].angle = max(min(self.stop_angles[1] + calculated_angles[1], 90+60), 90-40)
            pca.servo[self.servo_pins[2]].angle = max(min(self.stop_angles[2] + calculated_angles[2], 90+40), 90-40)
            pca.servo[self.servo_pins[3]].angle = max(min(self.stop_angles[3] + calculated_angles[3], 90+40), 90-40)

            if delay > 0:
                time.sleep(delay)
        except Exception as e:  
            print("I2C TIMEOUT!")
            debug(["INITIALISATION", "MOTORS", f"{e}"], [24, 15, 50])
            print()
            
            raise e
        
    def pause(self) -> None:
        self.run(0, 0)
        input()