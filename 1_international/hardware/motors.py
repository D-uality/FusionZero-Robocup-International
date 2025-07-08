from core.shared_imports import time, ServoKit, socket, getpass, math, mp, operator, np
from core.utilities import debug, show

class Motors():
    def __init__(self, camera) -> None:
        self.pca = ServoKit(channels=16)
        self.servo_pins = [14, 13, 12, 10]

        self.camera = camera

        username = getpass.getuser()
        hostname = socket.gethostname()
        self.user_at_host = f"{username}@{hostname}"
        
        if self.user_at_host == "frederick@raspberrypi":
            self.servo_pins = [14, 13, 12, 10]
            self.stop_angles = [97, 96, 96, 97]
            
            self.negative_gradients  = [-0.96727, -0.90880, -0.90704, -0.77515]
            self.negative_intercepts = [5.54149, 5.49693, 5.25736, 5.51568]
            self.positive_gradients  = [-1.00445, -0.93679, -0.72308, -0.78219]
            self.positive_intercepts = [-5.55059, -5.25260, -5.53597, -5.43908]

        elif self.user_at_host == "aidan@fusionzero":
            self.servo_pins = [14, 13, 12, 10]
            self.stop_angles = [88, 89, 88, 89]
            
            self.negative_gradients = [0.70106, 0.70440, 0.66426, 0.69425]
            self.negative_intercepts = [-6.8834, -4.9707, -5.0095, -4.9465]
            self.positive_gradients = [0.70924, 0.63035, 0.67858, 0.69916]
            self.positive_intercepts = [4.1509, 6.5548, 3.5948, 4.2287]
            
        else:
            print(self.user_at_host)
            raise ValueError(f"Unknown hostname: {hostname}")
       
        for i in range(0, 4):
            self.pca.servo[self.servo_pins[i]].angle = self.stop_angles[i]

        
        self.last_run = mp.Array('f', 2)   # [left, right] rad/s equiv.
        debug(["INITIALISATION", "MOTORS", "✓"], [25, 25, 50])
    
    def run(self, v1: float, v2: float, delay: float = 0) -> None:
        self.last_run[0] = v1
        self.last_run[1] = v2
        
        if self.user_at_host == "frederick@raspberrypi": v1, v2 = v2, v1
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
            else:                                    calculated_angles[i] = 0

            calculated_angles[i] = max(min(calculated_angles[i], 90), -90)
        
        try:
            self.pca.servo[self.servo_pins[0]].angle = max(min(self.stop_angles[0] + calculated_angles[0], 90+40), 90-40)
            self.pca.servo[self.servo_pins[1]].angle = max(min(self.stop_angles[1] + calculated_angles[1], 90+40), 90-40)
            self.pca.servo[self.servo_pins[2]].angle = max(min(self.stop_angles[2] + calculated_angles[2], 90+40), 90-40)
            self.pca.servo[self.servo_pins[3]].angle = max(min(self.stop_angles[3] + calculated_angles[3], 90+40), 90-40)

            if delay > 0:
                capture_start = time.perf_counter()
                
                while True:
                    if delay > 0.2: show(np.uint8(self.camera.capture_array()), display=self.camera.X11, name="line")
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
                # debug([f"{text}", f"{value:.2f}", f"{target_value:.2f}"], [24, 10, 10])
                # print()
                if comparison_function(value, target_value): break

        self.run(0, 0)
        
    def ease(self, v: float, start_angle: float, end_angle: float, step: int, max_time: float):
        angles = list(range(start_angle, end_angle, step))
        N = len(angles)
        if N < 2: raise ValueError("Need at least two angles for easing profile.")

        # Calculate weights using a sine curve for ease-in, ease-out
        weights = [math.sin(math.pi * i / (N - 1)) for i in range(N)]
        total_weight = sum(weights)
        normalized_weights = [w / total_weight for w in weights]
        times = [w * max_time for w in normalized_weights]

        for angle, t in zip(angles, times):
            self.steer(v, angle, t)

    def pause(self) -> None:
        self.run(0, 0)
        input()