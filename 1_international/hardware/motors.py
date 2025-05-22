from core.shared_imports import time, ServoKit, operator, socket, getpass, np
from core.utilities import debug, show

pca = ServoKit(channels=16)

class Motors():
    def __init__(self, camera) -> None:
        self.servo_pins = [14, 13, 12, 10]

        self.camera = camera
        
        username = getpass.getuser()
        hostname = socket.gethostname()
        user_at_host = f"{username}@{hostname}"
        
        if user_at_host == "frederick@raspberrypi":
            self.stop_angles = [88, 89, 88, 89]
            self.negative_gradients = [0.70106, 0.70440, 0.66426, 0.69425]
            self.negative_intercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
            self.positive_gradients = [0.70924, 0.63035, 0.67858, 0.69916]
            self.positive_intercepts = [4.1509, 6.5548, 3.5948, 4.2287]
            
        elif user_at_host == "aidan@fusionzero":
            self.stop_angles = [88, 89, 88, 89]
            self.negative_gradients = [0.70106, 0.70440, 0.66426, 0.69425]
            self.negative_intercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
            self.positive_gradients = [0.70924, 0.63035, 0.67858, 0.69916]
            self.positive_intercepts = [4.1509, 6.5548, 3.5948, 4.2287]
            
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
            pca.servo[self.servo_pins[0]].angle = max(min(self.stop_angles[0] + calculated_angles[0], 90+40), 90-40)
            pca.servo[self.servo_pins[1]].angle = max(min(self.stop_angles[1] + calculated_angles[1], 90+40), 90-40)
            pca.servo[self.servo_pins[2]].angle = max(min(self.stop_angles[2] + calculated_angles[2], 90+40), 90-40)
            pca.servo[self.servo_pins[3]].angle = max(min(self.stop_angles[3] + calculated_angles[3], 90+40), 90-40)

            if delay > 0:
                capture_start = time.perf_counter()
                while True:
                    show(np.uint8(self.camera.capture_array()), display=self.camera.X11, name="line")
                    if delay - (time.perf_counter() - capture_start) < 0: break
        except Exception as e:  
            print("I2C TIMEOUT!")
            debug(["INITIALISATION", "MOTORS", f"{e}"], [24, 15, 50])
            print()
    
    def run_until(self, v1: float, v2: float, trigger_function: callable, index: int, comparison: str, target_value: int, text: str = "") -> None:
        if   comparison == "==": comparison_function = operator.eq
        elif comparison == "<=": comparison_function = operator.le
        elif comparison == ">=": comparison_function = operator.ge
        elif comparison == "!=": comparison_function = operator.ne

        while True:
            value = trigger_function()[index]
            if value is not None: break
            
        while True:
            value = trigger_function()[index]
            self.run(v1, v2)
            
            if value is not None: 
                debug([f"{text}", f"{value:.2f}", f"{target_value:.2f}"], [24, 10, 10])
                print()
                if comparison_function(value, target_value): break

        self.run(0, 0)
        
    def pause(self) -> None:
        self.run(0, 0)
        input()