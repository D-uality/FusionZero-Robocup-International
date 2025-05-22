from core.shared_imports import time, np, cv2, Optional, YOLO
from hardware.robot import *
from core.utilities import *

start_display()

class EvacuationState():
    def __init__(self):
        self.victims_saved = 0
        self.X11 = True
        self.approach_distance = 7.5
        self.base_speed = 30
        
class Search():
    def __init__(self):
        self.debug = False
        
        self.model_path = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_models/dead_edgetpu.tflite"
        self.input_shape = 640
        self.model = YOLO(self.model_path, task='detect')
        self.confidence_threshold = 0.3
    
    def classic_live(self, image: np.ndarray, last_x: Optional[int] = None) -> list[np.ndarray, Optional[int]]:
        spectral_threshold = 200

        kernal_size = 7
        spectral_highlights = cv2.inRange(image, (spectral_threshold, spectral_threshold, spectral_threshold), (255, 255, 255))
        spectral_highlights = cv2.dilate(spectral_highlights, np.ones((kernal_size, kernal_size), np.uint8), iterations=1)

        contours, _ = cv2.findContours(spectral_highlights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours: return [None, image]

        valid_contours = []
        for contour in contours:
            _, y, w, h = cv2.boundingRect(contour)
            if self.debug: print(cv2.contourArea(contour))
            
            # look for contours in the top quarter
            if (y + h/2 < 50
                and cv2.contourArea(contour) < 1500):
                valid_contours.append(contour)
        
        if len(valid_contours) == 0: return [None, image]

        largest_contour = max(valid_contours, key=cv2.contourArea)
        x, _, w, _ = cv2.boundingRect(largest_contour)
        
        if evac_state.X11: cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
        return [int(x + w/2), image]
    
    def ai_dead(self, image:np.ndarray, last_x: Optional[int] = None) -> list[np.ndarray, Optional[int]]:
        results = self.model(image, imgsz=self.input_shape, conf=self.confidence_threshold, verbose=False)
        
        xywh = results[0].boxes.xywh
        cx_list = xywh[:, 0].cpu().numpy().tolist()
        
        if not cx_list: return [None, image]
        x = cx_list[0] if last_x is None else min(cx_list, key=lambda cx: abs(cx - last_x))
        
        image = results[0].plot()        
        return [int(x), image]

class WallFollow():
    OFFSET = 3

    def __init__(self, kP: float, base_speed: int):
        self.kP = kP
        self.base_speed = base_speed

    def check_touch(self) -> None:
        touch_values = touch_sensors.read()
        
        if sum(touch_values) != 2:
            motors.run(-self.base_speed, -self.base_speed, 0.3)
            motors.run(-self.base_speed,  self.base_speed, 1.3)

    def update(self) -> None:
        self.check_touch()
        
        distance = laser_sensors.read([0])[0]
        if distance is None: return None

        error = distance - self.OFFSET
        raw_turn = self.kP * error

        effective_turn = raw_turn * (1 + 7 / max(distance, self.OFFSET))

        max_turn = self.base_speed * 0.5
        effective_turn = min(max(effective_turn, -max_turn), max_turn)

        left_speed  = self.base_speed + effective_turn
        right_speed = self.base_speed - effective_turn

        motors.run(left_speed, right_speed)
        debug([f"{error:.2f}", f"{raw_turn:.2f}", f"{left_speed:.2f} {right_speed:.2f}"],[30, 20, 10])
        
def locate(searcher: Search) -> list[int, str]:
    wall_follower = WallFollow(kP=1.2, base_speed=30)
    base_speed = 30
    
    while True:
        image = evac_camera.capture_image()
        touch_values = touch_sensors.read()
        
        if sum(touch_values) != 2:
            motors.run(-base_speed, -base_speed, 0.5)
            motors.run( base_speed, -base_speed, 0.8)
        
        live_x, _ = searcher.classic_live(image)
        if live_x is not None: return [live_x, "live"]
        
        dead_x, _ = searcher.ai_dead(image)
        if dead_x is not None: return [dead_x, "dead"]
        
        motors.run(base_speed, base_speed)
        # wall_follower.update()
        if evac_state.X11: show(image, "image")

def route(searcher: Search, last_x: int, search_type: str) -> bool:
    base_speed = 30 if search_type == "live" else 25
    kP = 0.2
    min_distance = 0
    
    while True:
        distance = laser_sensors.read([1])[0]
        image = evac_camera.capture_image()
        
        if search_type == "live": x, image = searcher.classic_live(image, last_x)
        else:                     x, image = searcher.ai_dead(image, last_x)
        
        if x is None: return False
        if distance is None: continue
        if distance < evac_state.approach_distance: break
        
        min_distance = min(min_distance, distance)
        scalar = 0.5 + (distance - evac_state.approach_distance) * (0.5 / 85)
        
        error = evac_camera.width / 2 - x
        turn = kP * error
        
        v1 = scalar * base_speed - turn
        v2 = scalar * base_speed + turn
        
        if search_type == "live":
            motors.run(v1, v2)
        else:
            delay = 0.15 if error > 10 else 0.3
            v1 = min(40, max(v1, -10))
            v2 = min(40, max(v2, -10))

            motors.run(v1, v2, delay)
            motors.run(0, 0)
        
        debug( [ f"{distance}", f"{search_type} {x} {last_x}", f"{error} {turn:.2f}", f"{v1:.2f} {v2:.2f}"], [5, 15, 10, 10] )
        if evac_state.X11: show(image, "image")
        last_x = x
        
    return True

def align(searcher: Search, search_type: str, centre_tolorance: int, distance_tolorance) -> bool:
    last_x = None
    # Align to centre
    # while True:
    #     image = evac_camera.capture_image()
        
    #     if search_type == "live": x, image = searcher.classic_live(image, last_x)
    #     else:                     x, image = searcher.ai_dead(image, last_x)
        
    #     if x is None: return False
        
    #     if   x > int(evac_camera.width / 2) + centre_tolorance: motors.run( 30, -30, 0.005)
    #     elif x < int(evac_camera.width / 2) - centre_tolorance: motors.run(-30,  30, 0.005)
    #     else:                                                   break
        
    #     motors.run(0, 0, 0.005)
        
    #     debug( ["ALIGNING [CENTRE]", f"{x}"], [25, 15])
    #     if evac_state.X11: show(image, "image")
    #     last_x = x
        
    while True:
        initial_distance = laser_sensors.read([1])[0]
        if initial_distance is not None: break
    
    v1, v2 = -evac_state.base_speed, evac_state.base_speed
    pass_values = []
    
    while True:
        distance = laser_sensors.read([1])[0]
        pass_values.append(distance)
        
        motors.run(v1, v2, 0.003)
        motors.run(0, 0, 0.1)
        
        print(pass_values[-3:])
        
        if distance > initial_distance + 5:
            v1, v2 = v2, v1
            pass_values = []
            
        if len(pass_values) >= 3:
            if pass_values[-2] > pass_values[-3] and pass_values[-2] > pass_values[-1]:
                break
        
    # Move to set distance
    while True:
        distance = laser_sensors.read([1])[0]
        
        if   distance > evac_state.approach_distance + distance_tolorance: motors.run( 20,  20, 0.003)
        elif distance < evac_state.approach_distance - distance_tolorance: motors.run(-20, -20, 0.002)
        else:                                                              break
        
        motors.run(0, 0, 0.1)
        
        debug( ["ALIGNING [DISTANCE]", f"{distance}"], [25, 15])
        
    return True

def grab() -> bool:
    # Fill the avaliable slot
    
    if claw.spaces[0] == "":
        # Left
        motors.run(30, 0, 0.3)
        motors.run(0, 0)
        
        # Cup
        claw.close(180)
        claw.lift(0, 0.02)
        time.sleep(0.3)
        
        # Lift
        claw.close(90)
        claw.lift(180, 0.02)
        
        claw.spaces[0] = "live"
        
    elif claw.spaces[1] == "":
        # Right
        motors.run(0, 30, 0.3)
        motors.run(0, 0)
        
        # Cup
        claw.close(0)
        claw.lift(0, 0.02)
        time.sleep(0.3)
        
        # Lift
        claw.close(90)
        claw.lift(180, 0.02)
        
        claw.spaces[0] = "live"
    else: return False

evac_state = EvacuationState()

def main() -> None:
    searcher = Search()
    
    while True:
        x, search_type = locate(searcher)
        
        route_success = route(searcher, x, search_type)
        if not route_success: continue
        
        motors.run(0, 0, 1)

        align_success = align(searcher, search_type, 5, 0.2)
        if not align_success:
            print("FAILED ALIGNMENT")
            motors.pause()
            continue
        
        print("ALIGN SUCCESS!")
    
        motors.run(0, 0, 1)
        
        grab()
    
    # motors.run(30, -30)