from core.shared_imports import time, np, cv2, Optional, YOLO
from hardware.robot import *
from core.utilities import *

start_display()

class EvacuationState():
    def __init__(self):
        self.X11 = True
        
        self.victims_saved = 0
        self.base_speed = 30
        
        self.approach_distance = 7
        self.align_distance = 4
        
class Search():
    def __init__(self):
        self.debug = False
        
        self.model_path = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_models/dead_edgetpu.tflite"
        self.input_shape = 640
        self.model = YOLO(self.model_path, task='detect')
        self.confidence_threshold = 0.3
    
    def classic_live(self, image: np.ndarray, last_x: Optional[int]) -> list[np.ndarray, Optional[int]]:
        spectral_threshold = 230
        kernel_size = 7

        if last_x is not None:
            x0 = max(0, last_x - 75)
            x1 = min(image.shape[1], last_x + 75)
            image = image[:, x0:x1]

        spectral_highlights = cv2.inRange(image, (spectral_threshold, spectral_threshold, spectral_threshold), (255, 255, 255))
        spectral_highlights = cv2.dilate(spectral_highlights, np.ones((kernel_size, kernel_size), np.uint8), iterations=1)

        contours, _ = cv2.findContours(spectral_highlights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return [None, image]

        valid = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            
            if y + h/2 > 5 and y + h/2 < 150 and area < 3000:
                cx = x + w/2
                valid.append((contour, cx))

        if not valid: return [None, image]

        if last_x is None:
            largest = max(valid, key=lambda t: cv2.contourArea(t[0]))[0]
            bx, _, bw, _ = cv2.boundingRect(largest)
            
        else:
            best = min(valid, key=lambda t: abs(t[1]))[0]
            bx, _, bw, _ = cv2.boundingRect(best)

        if evac_state.X11: cv2.drawContours(image, [best if last_x is not None else largest], -1, (0, 255, 0), 1)

        center_x = bx + bw/2
        if last_x is not None: center_x += (last_x - 50)
        
        return [int(center_x), image]
    
    def ai_dead(self, image:np.ndarray, last_x: Optional[int]) -> list[np.ndarray, Optional[int]]:
        results = self.model(image, imgsz=self.input_shape, conf=self.confidence_threshold, verbose=False)
        
        xywh = results[0].boxes.xywh
        cx_list = xywh[:, 0].cpu().numpy().tolist()
        
        if not cx_list: return [None, image]
        x = cx_list[0] if last_x is None else min(cx_list, key=lambda cx: abs(cx - last_x))
        
        image = results[0].plot()
        return [int(x), image]
    
    def red_triangle(self, image: np.ndarray) -> Optional[int]:
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Adjusting: V_min | H_min=0, H_max=10, S_min=230, S_max=255, V_min=46, V_max=255
        # Adjusting: S_min | H_min=170, H_max=179, S_min=230, S_max=255, V_min=0, V_max=255
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_lower = cv2.inRange(hsv_image, (0, 230, 46), (10, 255, 255))
        mask_upper = cv2.inRange(hsv_image, (170, 230, 0), (179, 255, 255))
        mask = cv2.bitwise_or(mask_lower, mask_upper)
        mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If no contours are found, return None
        if not contours: return None, None, None, None
        largest_contour = max(contours, key=cv2.contourArea)

        # If the largest contour is too small, return None
        if cv2.contourArea(largest_contour) < 1200: return None, None, None, None

        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        if h > w: return None

        # Display debug information
        if evac_state.X11:
            cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
            cv2.circle(image, (int(x + w / 2), int(y + h / 2)), 3, (255, 0, 0), 1)

        return int(x + w/2)
        
    def green_triangle(self, image: np.ndarray) -> Optional[int]:
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Adjusting: V_min | H_min=0, H_max=10, S_min=230, S_max=255, V_min=46, V_max=255
        # Adjusting: S_min | H_min=170, H_max=179, S_min=230, S_max=255, V_min=0, V_max=255
        mask = cv2.inRange(hsv_image, (50, 120, 15), (70, 255, 255))
        mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If no contours are found, return None
        if not contours: return None, None, None, None
        largest_contour = max(contours, key=cv2.contourArea)

        # If the largest contour is too small, return None
        if cv2.contourArea(largest_contour) < 1200: return None, None, None, None

        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        if h > w: return None

        # Display debug information
        if evac_state.X11:
            cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
            cv2.circle(image, (int(x + w / 2), int(y + h / 2)), 3, (255, 0, 0), 1)

        return int(x + w/2)

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
        
def locate(searcher: Search) -> tuple[int, str]:
    wall_follower = WallFollow(kP=1.2, base_speed=30)
    base_speed = 30
    
    while True:
        image = evac_camera.capture_image()
        touch_values = touch_sensors.read()
        
        if sum(touch_values) != 2:
            motors.run(-base_speed, -base_speed, 0.5)
            motors.run( base_speed, -base_speed, 1.3)
        
        if   claw.spaces[0] == "" and claw.spaces[1] == "":
            live_x, _ = searcher.classic_live(image)
            if live_x is not None: return live_x, "live"
            
            dead_x, _ = searcher.ai_dead(image)
            if dead_x is not None: return dead_x, "dead"
        
        elif claw.spaces[0] != "" and claw.spaces[1] != "":
            if evac_state.victims_saved <= 2:
                green_x = searcher.green_triangle(image)
                if green_x is not None: return green_x
                
            else:
                red_x = searcher.red_triangle(image)
                if red_x is not None: return red_x
        
        else:
            if evac_state.victims_saved <= 2 and "live" in claw.spaces:
                green_x = searcher.green_triangle(image)
                if green_x is not None: return green_x
                
            elif "dead" in claw.spaces:
                red_x = searcher.red_triangle(image)
                if red_x is not None: return red_x
            
            live_x, _ = searcher.classic_live(image)
            if live_x is not None: return live_x, "live"
            
            dead_x, _ = searcher.ai_dead(image)
            if dead_x is not None: return dead_x, "dead"
            
        motors.run(base_speed, base_speed)
        # wall_follower.update()
        if evac_state.X11: show(image, "image")
        debug([f"LOCATING"], [15])

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
            delay = 0.15
            v1 = min(40, max(v1, -10))
            v2 = min(40, max(v2, -10))

            motors.run(v1, v2, delay)
            motors.run(0, 0)
        
        debug( [ f"{distance}", f"{search_type} {x} {last_x}", f"{error} {turn:.2f}", f"{v1:.2f} {v2:.2f}"], [5, 15, 10, 10] )
        if evac_state.X11: show(image, "image")
        last_x = x
        
    return True

def find_centre(centre_tolorance: int, direction: int = 1):
    time_step = 0.003
    initial_distance = 0
    while True:
        initial_distance = laser_sensors.read([1])[0]
        if initial_distance is not None: break
            
    distances = []
    
    while True:
        distance = laser_sensors.read([1])[0]
        if distance is None: continue
        
        if distance not in distances: distances.append(distance)
        
        motors.run(-20 * direction, 20 * direction, time_step)
        motors.run(              0,              0, time_step * 3)
        
        if len(distances) >= 3:
            if distances[-1] > distances[-2] and distances[-3] > distances[-2]: return True
        if distance > initial_distance + centre_tolorance:                      return False
        
        debug( ["ALIGNING [CENTRE]", "TURNING TILL NOT", f"{distance}"], [15, 15, 15])

def align(centre_tolorance: int, distance_tolorance: int) -> bool:
    time_step = 0.002
    
    while True:
        distance = laser_sensors.read([1])[0]
        if distance is None: continue
        
        if   distance > evac_state.align_distance + distance_tolorance: motors.run( 20,  20, time_step)
        elif distance < evac_state.align_distance - distance_tolorance: motors.run(-20, -20, time_step)
        else:                                                           break
        
        motors.run(0, 0, time_step * 3)
        
        debug( ["ALIGNING [DISTANCE]", f"{distance}"], [25, 15])
    
    direction = 1
    while not find_centre(centre_tolorance, direction):
        direction *= -1
    
    motors.run(0, 0, 0.3)
        
    while True:
        distance = laser_sensors.read([1])[0]
        if distance is None: continue
        
        if   distance > evac_state.align_distance + distance_tolorance: motors.run( 20,  20, time_step)
        elif distance < evac_state.align_distance - distance_tolorance: motors.run(-20, -20, time_step)
        else:                                                           break
        
        motors.run(0, 0, time_step * 2)
        
        debug( ["ALIGNING [DISTANCE]", f"{distance}"], [25, 15])
        
    return True

def grab() -> bool:
    # Fill the avaliable slot
    motors.run(-evac_state.base_speed, -evac_state.base_speed, 0.3)
    
    if claw.spaces[0] == "":
        # Left
        motors.run(evac_state.base_speed, -evac_state.base_speed, 0.25)
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
        motors.run(-evac_state.base_speed, evac_state.base_speed, 0.25)
        motors.run(0, 0)
        
        # Cup
        claw.close(0)
        claw.lift(0, 0.02)
        time.sleep(0.3)
        
        # Lift
        claw.close(90)
        claw.lift(180, 0.02)
        
        claw.spaces[1] = "live"
    else: return False

evac_state = EvacuationState()

def main() -> None:
    searcher = Search()
    
    # Warmup Camera
    for _ in range(0, 5): image = evac_camera.capture_image()
    # Warup TPU
    searcher.ai_dead(image)
    
    while True:
        x, search_type = locate(searcher)
        
        route_success = route(searcher, x, search_type)
        if not route_success: continue
        
        motors.run(0, 0, 1)

        align_success = align(10, 0.1)
        if not align_success: continue
            
        motors.run(0, 0, 1)
        
        grab()