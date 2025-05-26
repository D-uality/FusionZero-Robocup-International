from core.shared_imports import time, np, cv2, Optional, YOLO, randint
from hardware.robot import *
from core.utilities import *

start_display()

class EvacuationState():
    def __init__(self):
        self.X11 = True
        self.debug = True
        
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
    
    def classic_live(self, image: np.ndarray, last_x: Optional[int]) -> Optional[int]:
        THRESHOLD = 230
        KERNAL_SIZE = 7
        CROP_SIZE = 75

        # Crop image to approximate region for higher performance
        if last_x is not None:
            x_lower = max(                0, last_x - CROP_SIZE)
            x_upper = min(evac_camera.width, last_x + CROP_SIZE)
            image = image[:, x_lower : x_upper]

        # Filter for spectral highlights
        spectral_highlights = cv2.inRange(image, (THRESHOLD, THRESHOLD, THRESHOLD), (255, 255, 255))
        spectral_highlights = cv2.dilate(spectral_highlights, np.ones((KERNAL_SIZE, KERNAL_SIZE), np.uint8), iterations=1)

        contours, _ = cv2.findContours(spectral_highlights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None

        # Validate contours based on: area, y-position
        valid = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            
            if y + h/2 > 5 and y + h/2 < 150 and area < 3000:
                cx = x + w/2
                valid.append((contour, cx))
        if len(valid) == 0: return None

        # Find largest contour/closest contour
        best_contour = max(valid, key=lambda t: cv2.contourArea(t[0]))[0] if last_x is None else min(valid, key=lambda t: abs(t[1]))[0]
        x, _, w, _ = cv2.boundingRect(best_contour)
        center_x = x + w/2 if last_x is None else (x + w/2) + (last_x - CROP_SIZE)

        if evac_state.X11:   cv2.drawContours(image, [best_contour], -1, (0, 255, 0), 1)
        
        # Find centre x 
        return int(center_x)
    
    def ai_dead(self, image:np.ndarray, last_x: Optional[int]) -> Optional[int]:
        results = self.model(image, imgsz=self.input_shape, conf=self.confidence_threshold, verbose=False)
        
        xywh = results[0].boxes.xywh
        cx_list = xywh[:, 0].cpu().numpy().tolist()
        
        if not cx_list: return None
        x = cx_list[0] if last_x is None else min(cx_list, key=lambda cx: abs(cx - last_x))
        
        image = results[0].plot()
        return int(x)
    
    def triangle(self, image: np.ndarray, triangle: str) -> Optional[int]:
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if triangle == "red":
            # Adjusting: V_min | H_min=0, H_max=10, S_min=230, S_max=255, V_min=46, V_max=255
            # Adjusting: S_min | H_min=170, H_max=179, S_min=230, S_max=255, V_min=0, V_max=255
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask_lower = cv2.inRange(hsv_image, (0, 230, 46), (10, 255, 255))
            mask_upper = cv2.inRange(hsv_image, (170, 230, 0), (179, 255, 255))
            mask = cv2.bitwise_or(mask_lower, mask_upper)
            mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
        
        else:
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

class Movement():
    def __init__(self):
        self.offset = 3
        
        self.state = "forwards"
        self.t0 = time.perf_counter()
            
    def random(self) -> tuple[int]:
        # Set speeds
        v1 = v2 = 0
        if self.state == "forwards":
            v1 =  evac_state.base_speed
            v2 =  evac_state.base_speed
        
        elif self.state == "turning":
            v1 =  evac_state.base_speed
            v2 = -evac_state.base_speed
        
        motors.run(v1, v2)
        
        # Update state
        if self.state == "forwards" and sum(touch_sensors.read()) != 2:
            self.state = "turning"
            self.t0 = time.perf_counter()
            
        elif self.state == "turning" and time.perf_counter() - time.perf_counter() - self.t0 > 1.5:
            self.state = "forwards"
            
        return v1, v2
    
    def route(self, kP: float, x: int, distance: float, last_distance: float, time_step: float = 0) -> tuple[int]:
        Y_INTERCEPT = 0.5
        MAXIMUM = 85
        MAX_VELOCITY = 40
        MIN_VELOCITY = -10
        
        min_distance = min(distance, last_distance)
        
        scalar = (Y_INTERCEPT / MAXIMUM) * min_distance + Y_INTERCEPT 
        error = evac_camera.width / 2 - x
        
        turn = (kP * error) * scalar
        
        v1 = evac_state.base_speed + turn
        v2 = evac_state.base_speed - turn
        
        v1 = min(MAX_VELOCITY, max(v1, MIN_VELOCITY))
        v2 = min(MAX_VELOCITY, max(v2, MIN_VELOCITY))
        
        motors.run(v1, v2, time_step)
        if time_step != 0: motors.run(0, 0)
        
        return v1, v2
    
    def wall_follow(self) -> tuple[int]:
        touch_values = touch_sensors.read()
        
        if sum(touch_values) != 2:
            motors.run(-self.base_speed, -self.base_speed, 0.3)
            motors.run(-self.base_speed,  self.base_speed, 1.3)
        
        distance = laser_sensors.read([0])[0]
        if distance is None: return None

        error = distance - self.OFFSET
        raw_turn = self.kP * error

        effective_turn = raw_turn * (1 + 7 / max(distance, self.OFFSET))

        max_turn = self.base_speed * 0.5
        effective_turn = min(max(effective_turn, -max_turn), max_turn)

        v1 = self.base_speed + effective_turn
        v2 = self.base_speed - effective_turn

        motors.run(v1, v2)
        debug([f"{error:.2f}", f"{raw_turn:.2f}", f"{v1:.2f} {v2:.2f}"],[30, 20, 10])
        
        return v1, v2

def analyse(image: np.ndarray, search_type: str = "default", last_x: Optional[int] = None) -> tuple[Optional[int], str]:    
    victims_enable = True
    triangles_enable = True
    
    # Determine what to search for based on claw state
    if claw.spaces[0] == "" and claw.spaces == "":   triangles_enable = False
    elif claw.spaces[0] != "" and claw.spaces != "": victims_enable   = False
    
    if triangles_enable and search_type in ["red", "green", "default"]:
        x = search.triangle(image, "green" if evac_state.victims_saved <= 2 else "red")
        if x is not None: return x, "green" if evac_state.victims_saved <= 2 else "red"
    
    if victims_enable:
        if search_type in ["live", "default"]:
            live_x = search.classic_live(image, last_x)
            if live_x is not None: return live_x, "live"
    
        if search_type in ["dead", "default"]:
            dead_x = search.ai_dead(image, last_x)
            if dead_x is not None: return dead_x, "dead"
        
    return None, ""

def locate() -> tuple[int, str]:
    movement = Movement()
    
    while True:
        image = evac_camera.capture_image()
        
        # Exit if target found
        x, search_type = analyse(search, image)
        if x is not None: return x, search_type
            
        v1, v2 = movement.random()
        
        if evac_state.X11:   show(image, "image")
        if evac_state.debug: debug( [f"LOCATING", f"{v1} {v2}"], [15, 15] )

def route(last_x: int, search_type: str) -> bool:
    last_x = 0
    last_distance = 100
    
    while True:
        # Take measurements
        distance = laser_sensors.read([1])[0]
        image = evac_camera.capture_image()
        
        # Exit conditions
        if   search_type in ["live", "dead"] and distance < evac_state.approach_distance: break
        elif search_type in ["red", "green"] and sum(touch_sensors.read()) == 0:          break        
        
        # Error handling
        if distance is None: continue
        if        x is None: return False
        
        # Route with kP
        min_distance_found = min(min_distance_found, distance)
        x, search_type = analyse(image, search_type, last_x)
        
        time_step = 0.15 if search_type == "dead" else 0
        v1, v2, last_distance = movement.route(1.3, x, distance, last_distance, time_step)
        
        last_x = x
        
        debug( ["ROUTING", f"{search_type}", f"Distance: {distance:.2f}", f"x: {x} {last_x}", f"{v1:.2f} {v2:.2f}"], [15, 15, 20, 20] )

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
search = Search()
movement = Movement()

def main() -> None:    
    # Warmup camera and TPU
    for _ in range(0, 5): image = evac_camera.capture_image()
    search.ai_dead(image)
    
    while True:
        x, search_type = locate(search)
        
        route_success = route(search_type, x)
        if not route_success: continue
        
        motors.run(0, 0, 1)

        align_success = align(10, 0.1)
        if not align_success: continue
            
        motors.run(0, 0, 1)
        
        grab()