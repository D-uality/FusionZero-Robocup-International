from core.shared_imports import time, np, cv2, Optional, YOLO, math
from hardware.robot import *
from core.utilities import *

start_display()

class EvacuationState():
    def __init__(self):
        self.X11 = True
        self.debug = True
        
        self.victims_saved = 0
        self.base_speed  = 35
        self.fast_speed  = 45
        self.align_speed = 30
        self.grab_speed  = 30
        
        self.live_approach_distance     = 7
        self.dead_approach_distance     = 7
        self.triangle_approach_distance = 6.5
        self.align_distance = 4
        
        self.minimum_align_distance = 20
        
class Search():
    def __init__(self):
        self.debug = True
        
        self.model_path = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_models/dead_edgetpu.tflite"
        self.input_shape = 640
        self.model = YOLO(self.model_path, task='detect')
        self.confidence_threshold = 0.3
    
    def classic_live(self, image: np.ndarray, last_x: Optional[int]) -> Optional[int]:
        THRESHOLD = 245
        KERNEL_SIZE = 7
        CROP_SIZE = 100

        # Ensure image is not None
        if image is None or image.size == 0: return None
        
        # Crop according to last x
        if last_x is not None:
            x_lower = max(             0, last_x - CROP_SIZE)
            x_upper = min(image.shape[1], last_x + CROP_SIZE)
            
            if x_upper > x_lower: image = image[:, x_lower : x_upper]
            
        # Filter for spectral highlights
        spectral_highlights = cv2.inRange(image, (THRESHOLD, THRESHOLD, THRESHOLD), (255, 255, 255))
        spectral_highlights = cv2.dilate (spectral_highlights, np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8), iterations=1)

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
        center_x = x + w/2 if last_x is None else (x + w/2) + last_x

        if evac_state.X11:   cv2.drawContours(image, [best_contour], -1, (0, 255, 0), 1)
        
        # Find centre x 
        return int(center_x)
    
    def hough_dead(self, image: np.ndarray, last_x: Optional[int], distance: Optional[float] = None) -> Optional[int]:
        CROP_SIZE = 200
        
        DARK_BLACK = 35
        THRESHOLD_BLACK = 100
        THRESHOLD_SILVER = 160
        # INPAINT_RADIUS = 5
        KERNAL_SIZE_1 = 3
        KERNAL_SIZE_2 = 5
        KERNAL_SIZE_3 = 31
        
        DP =           1                               # "Resolution" of the accumulator
        MIN_DISTANCE = 300
        P1 =           55                              # Lower -> detect more circles   |  Higher -> detect less circles
        P2 =           25                              # Lower -> accepts more circles  |  Higher -> rejects more circles
        MIN_RADIUS =   20
        MAX_RADIUS =   230
        
        OFFSET = 20
        
        working_image = image.copy()
        
        # Ensure image is not None
        if working_image is None or working_image.size == 0: return None
        
        # Crop according to last x
        if last_x is not None:
            x_lower = max(                     0, last_x - CROP_SIZE)
            x_upper = min(working_image.shape[1], last_x + CROP_SIZE)
            
            if x_upper > x_lower: working_image = working_image[:, x_lower : x_upper]
        
        t0 = time.perf_counter()
        # Remove green, replace with grey
        hsv_image = cv2.cvtColor(working_image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_image, (50, 120, 50), (70, 255, 255))
        green_mask = cv2.dilate(green_mask, np.ones((KERNAL_SIZE_2, KERNAL_SIZE_2), np.uint8), iterations=1)
        working_image[green_mask > 0] = [41, 41, 41]
        
        t1 = time.perf_counter()
        # Remove spectral highlights
        # spectral_highlights = cv2.inRange(working_image, (250, 250, 250), (255, 255, 255))
        # spectral_highlights = cv2.dilate(spectral_highlights, np.ones((KERNAL_SIZE_2, KERNAL_SIZE_2), np.uint8), iterations=1)
        # working_image = cv2.inpaint(working_image, spectral_highlights, inpaintRadius=INPAINT_RADIUS, flags=cv2.INPAINT_TELEA)
        
        # Normalise white
        white = cv2.inRange(working_image, (THRESHOLD_SILVER, THRESHOLD_SILVER, THRESHOLD_SILVER), (255, 255, 255))
        white = cv2.dilate(white, np.ones((KERNAL_SIZE_1, KERNAL_SIZE_1), np.uint8), iterations=1)
        working_image[white > 0] = [160, 160, 160]
        
        t2 = time.perf_counter()
        # crop based on black mask
        mask  = cv2.inRange(working_image, (0, 0, 0), (THRESHOLD_BLACK, THRESHOLD_BLACK, THRESHOLD_BLACK))
        mask  = cv2.dilate(mask, np.ones((KERNAL_SIZE_3, KERNAL_SIZE_3), np.uint8), iterations=1)
        
        # Remove small black contours from mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_mask = np.zeros_like(mask)
        for contour in contours:
            if cv2.contourArea(contour) >= 6000:
                cv2.fillPoly(filtered_mask, [contour], 255)
        mask = filtered_mask
        
        # Search for only black parts of the image
        grey_image = cv2.cvtColor(working_image, cv2.COLOR_BGR2GRAY)
        grey_image = cv2.bitwise_and(grey_image, mask)

        if self.debug: show(grey_image, "grey_image")
        
        t3 = time.perf_counter()
        # Hough Circle Transform
        circles = cv2.HoughCircles(grey_image, cv2.HOUGH_GRADIENT, DP, MIN_DISTANCE, param1=P1, param2=P2, minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)
        if circles is None: return None
        
        t4 = time.perf_counter()
        # Process circles
        circles = np.round(circles[0, :]).astype("int")
        # Validate circles based on: colour, position, size
        valid = []
        for (x, y, r) in circles:
            # Clamp coordinates to image bounds
            x = max(0, min(x, working_image.shape[1] - 1))
            y = max(0, min(y, working_image.shape[0] - 1))
            
            # colour at x y on image must be black
            if (working_image[y, x][0] < DARK_BLACK and working_image[y, x][1] < DARK_BLACK and working_image[y, x][2] < DARK_BLACK
                # and y + r < working_image.shape[0] + OFFSET and y - r > -OFFSET
                and y > 10):
                valid.append((x, y, r))
        
        t5 = time.perf_counter()
        if len(valid) == 0: return None
        
        # Display on image
        for (x, y, r) in valid:
            cv2.circle(working_image, (x, y), r, (0, 255, 0), 1)
            cv2.circle(working_image, (x, y), 1, (0, 0, 255), 1)
        
        # Find the closest circle to last x, or the largest circle if last x is None
        if last_x is not None:
            closest_circle = min(valid, key=lambda circle: abs(circle[0] - (last_x - x_lower)))
            x = closest_circle[0] + x_lower
        else:
            largest_circle = max(valid, key=lambda circle: circle[2])
            x = largest_circle[0]
                
        if evac_state.X11 and self.debug:   show(working_image, "hough_circles")
        if self.debug: print(f"Green: {(t1-t0)*1000:.1f}ms | Spectral: {(t2-t1)*1000:.1f}ms | Mask: {(t3-t2)*1000:.1f}ms | Hough: {(t4-t3)*1000:.1f}ms | Validation: {(t5-t4)*1000:.1f}ms | Total: {(t5-t0)*1000:.1f}ms")

        return int(x)
    
    def classic_dead(self, image: np.ndarray, last_x: Optional[int]) -> Optional[int]:
        THRESHOLD = 45
        KERNAL_SIZE = 5
        CROP_SIZE = 100
        
        # Crop image to approximate region for higher performance
        if last_x is not None:
            x_lower = max(                0, last_x - CROP_SIZE)
            x_upper = min(evac_camera.width, last_x + CROP_SIZE)
            image = image[:, x_lower : x_upper]
        
        image = cv2.dilate(image, np.ones((KERNAL_SIZE, KERNAL_SIZE), np.uint8), iterations=1)
        mask = cv2.inRange(image, (0, 0, 0), (THRESHOLD, THRESHOLD, THRESHOLD))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None

        # Validate contours based on: area, y-position
        valid = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            
            if y + h/2 > 5 and y + h/2 < 150 and 100 < area < 20000:
                cx = x + w/2
                valid.append((contour, cx))
        if len(valid) == 0: return None

        # Find largest contour/closest contour
        best_contour = max(valid, key=lambda t: cv2.contourArea(t[0]))[0] if last_x is None else min(valid, key=lambda t: abs(t[1]))[0]
        x, _, w, _ = cv2.boundingRect(best_contour)
        center_x = x + w/2 if last_x is None else (x + w/2) + (last_x - CROP_SIZE)

        if evac_state.X11:   cv2.drawContours(image, [best_contour], -1, (0, 255, 0), 1)
        
        show(image, "dead_cropped")
        
        # Find centre x 
        return int(center_x)
         
    def ai_dead(self, image:np.ndarray, last_x: Optional[int]) -> Optional[int]:
        try:
            results = self.model(image, imgsz=self.input_shape, conf=self.confidence_threshold, verbose=False)
        except Exception as e:
            debug( [f"ERROR", f"TPU", f"{e}"], [30, 20, 50] )
            motors.run(0, 0, 1)
        
        xywh = results[0].boxes.xywh
        cx_list = xywh[:, 0].cpu().numpy().tolist()
        
        if not cx_list: return None
        x = cx_list[0] if last_x is None else min(cx_list, key=lambda cx: abs(cx - last_x))
        
        image = results[0].plot()
        return int(x)
    
    def triangle(self, image: np.ndarray, triangle: str) -> Optional[int]:
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if triangle == "red":
            mask_lower = cv2.inRange(hsv_image, (  0, 120, 0), ( 20, 255, 255))
            mask_upper = cv2.inRange(hsv_image, (160, 120, 0), (179, 255, 255))
            mask = cv2.bitwise_or(mask_lower, mask_upper)
        
        else:
            mask = cv2.inRange(hsv_image, (50, 120, 15), (70, 255, 255))
            
        mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If no contours are found, return None
        if not contours: return None
        
        if self.debug: 
            for contour in contours: print(cv2.contourArea(contour))
        
        largest_contour = max(contours, key=cv2.contourArea)

        # If the largest contour is too small, return None
        if cv2.contourArea(largest_contour) < 4500: return None

        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        if h > w or y + h/2 > evac_camera.height / 2: return None

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
        
        elif self.state == "touch":
            v1 =  evac_state.base_speed * 0.65
            v2 = -evac_state.base_speed * 0.65
        
        motors.run(v1, v2)
        
        # Update state
        if self.state == "forwards" and sum(touch_sensors.read()) != 2:
            motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.3)
            self.state = "touch"
            self.t0 = time.perf_counter()
            
        elif self.state == "touch" and time.perf_counter() - self.t0 > 1:
            self.state = "forwards"
            
        return v1, v2
    
    def route(self, kP: float, x: int, search_type: str, distance: float, last_distance: float, time_step: float = 0) -> tuple[int]:        
        Y_INTERCEPT = 0.5
        MAXIMUM = 100
        MAX_VELOCITY = 40
        MIN_VELOCITY = -10
        
        min_distance = min(distance, last_distance)
        
        scalar = (Y_INTERCEPT / MAXIMUM) * min_distance + Y_INTERCEPT if search_type in ["live", "dead"] else 1
        error = evac_camera.width / 2 - x

        turn = kP * error
        
        v1 = (evac_state.align_speed - turn if search_type in ["live", "dead"] else evac_state.fast_speed - turn) * scalar
        v2 = (evac_state.align_speed + turn if search_type in ["live", "dead"] else evac_state.fast_speed + turn) * scalar
        
        v1 = min(MAX_VELOCITY, max(v1, MIN_VELOCITY))
        v2 = min(MAX_VELOCITY, max(v2, MIN_VELOCITY))
        
        motors.run(v1, v2, time_step)
        if time_step != 0: motors.run(0, 0)
        
        return v1, v2, min_distance
    
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
        debug([f"{error:.2f}", f"{raw_turn:.2f}", f"{v1:.2f} {v2:.2f}"], [30, 20, 20])
        
        return v1, v2

def analyse(image: np.ndarray, search_type: str = "default", last_x: Optional[int] = None) -> tuple[Optional[int], str]:      
    live_enable  = False
    dead_enable  = False
    green_enable = False
    red_enable   = False
    
    claw.read()
    live_count  = claw.spaces.count("live")
    dead_count  = claw.spaces.count("dead")
    empty_count = 2 - live_count - dead_count
        
    if evac_state.victims_saved + live_count < 2 and empty_count != 0: live_enable  = True
    if dead_count < 1 and empty_count != 0:                            dead_enable  = True
    if live_count > 0:                                                 green_enable = True
    if dead_count == 1 and evac_state.victims_saved == 2:              red_enable   = True
            
    if search_type in ["live", "green", "default"] and live_enable:
        live_x = search.classic_live(image, last_x)
        if live_x is not None: return live_x, "live"

    if search_type in ["dead", "green", "default"] and dead_enable:
        dead_x = search.hough_dead(image, last_x)
        if dead_x is not None: return dead_x, "dead"
    
    if search_type in ["green", "default"] and green_enable:
        x  = search.triangle(image, "green")
        if x is not None: return x, "green"
    
    if search_type in ["red", "default"] and red_enable:
        x  = search.triangle(image, "red")
        if x is not None: return x, "red"
    
    return None, search_type

def locate() -> tuple[int, str]:
    movement = Movement()
    
    while True:
        image = evac_camera.capture_image()
        
        # Exit if target found
        x, search_type = analyse(image)
        if x is not None: return x, search_type
            
        v1, v2 = movement.random()
        
        if evac_state.X11:   show(image, "image")
        if evac_state.debug: debug( [f"LOCATING", f"{v1} {v2}", f"search: {search_type}", f"victims: {evac_state.victims_saved}", f"claw: {claw.spaces}"], [30, 20, 20, 20, 20] )

def route(last_x: int, search_type: str) -> bool:
    last_distance = 100
    
    while True:
        # Take measurements
        distance = laser_sensors.read([1])[0]
        image = evac_camera.capture_image()
        x, search_type = analyse(image, search_type, last_x)
        
        # Error handling
        if distance is None: continue
        if        x is None: return False
        
        # Exit conditions
        if   search_type in ["live"]         and distance < evac_state.live_approach_distance:     return True
        elif search_type in ["dead"]         and distance < evac_state.dead_approach_distance:     return True
        elif search_type in ["red", "green"] and distance < evac_state.triangle_approach_distance: break
        
        # Route with kP        
        # time_step = 0.15 if search_type == "dead" else 0
        time_step = 0
        v1, v2, last_distance = movement.route(0.2, x, search_type, distance, last_distance, time_step)
        
        last_x = x
        
        if evac_state.X11:   show(image, "image")
        if evac_state.debug: debug( ["ROUTING", f"{search_type}", f"Distance: {distance:.2f}", f"x: {x} {last_x}", f"{v1:.2f} {v2:.2f}"], [30, 20, 20, 20] )

    while True:
        if sum(touch_sensors.read()) == 0: break
        motors.run(evac_state.fast_speed, evac_state.fast_speed)
    
    return True

def forwards_align(distance_tolorance: float, time_step: float) -> bool:
    MAX_ALIGN_TIME = 5
    t0 = time.perf_counter()
    
    while True:
        distance = laser_sensors.read([1])[0]
        
        debug( ["ALIGNING [DISTANCE]", f"{distance}"], [30, 20])
        
        if distance is None:                          continue
        if distance > 30:                             return False
        if time.perf_counter() - t0 > MAX_ALIGN_TIME: return False
        
        if   distance > evac_state.align_distance + distance_tolorance: motors.run( evac_state.align_speed,  evac_state.align_speed, time_step)
        elif distance < evac_state.align_distance - distance_tolorance: motors.run(-evac_state.align_speed, -evac_state.align_speed, time_step)
        else:                                                           return True
        
        motors.run(0, 0, time_step * 3)
        
def sweep(centre_tolorance: float) -> bool:
    SWEEP_TIME = 0.7
    distances = []
    
    # Setup
    motors.run(-evac_state.base_speed * 0.85, evac_state.base_speed * 0.85, SWEEP_TIME)
    
    t0 = time.perf_counter()
    while True:
        distance = laser_sensors.read([1])[0]
        
        if time.perf_counter() - t0 > SWEEP_TIME * 2: break
        if distance is None: continue
        
        distances.append(distance)
        motors.run(evac_state.align_speed * 0.85, -evac_state.align_speed * 0.85)
        
    motors.run(0, 0)
    target_distance = min(distances)
    
    # Sweep past to the target distance
    t0 = time.perf_counter()
    while True:
        distance = laser_sensors.read([1])[0]
        
        if distance is None: continue
        if distance < target_distance + centre_tolorance: break
        if time.perf_counter() - t0 > SWEEP_TIME * 5: return False
        
        motors.run(-evac_state.align_speed * 0.85, evac_state.align_speed * 0.85)
    
    motors.run(-evac_state.align_speed * 0.85, evac_state.align_speed * 0.85, 0.3)
    
    # Sweep back to the target distance
    t0 = time.perf_counter()
    while True:
        distance = laser_sensors.read([1])[0]
        
        if distance is None: continue
        if distance < target_distance + centre_tolorance: break
        if time.perf_counter() - t0 > SWEEP_TIME * 5: return False
        
        motors.run(-evac_state.align_speed * 0.85, evac_state.align_speed * 0.85)
        
    # motors.run(0, 0, 0.1)
    # motors.run(evac_state.align_speed, -evac_state.align_speed, 0.025)
    
    return True

def align(centre_tolorance: float, distance_tolorance: float) -> bool:
    t0 = time.perf_counter()
    TIME_STEP = 0.025
    
    if not forwards_align(distance_tolorance, TIME_STEP): return False
    motors.run(0, 0, 0.3)
    if not forwards_align(distance_tolorance, TIME_STEP): return False
            
    if not sweep(centre_tolorance): return False
    
    if not forwards_align(distance_tolorance, TIME_STEP): return False
    motors.run(0, 0, 0.3)
    if not forwards_align(distance_tolorance, TIME_STEP): return False
    
    return True

def grab() -> bool:
    claw.read()
    if "" not in claw.spaces: return False
    
    # Left: -1, Right: 1
    insert = -1 if claw.spaces[0] == "" else 1
    
    debug( ["GRAB", "SETUP"], [30, 20] )
    # Setup
    motors.run(         -evac_state.grab_speed,         -evac_state.grab_speed, 0.25)
    motors.run(-evac_state.grab_speed * insert, evac_state.grab_speed * insert, 0.25)
    motors.run(0, 0)
    
    debug( ["GRAB", "CUP"], [30, 20] )
    # Cup
    claw.close(180 if insert == -1 else 0)
    claw.lift(0, 0.005)
    time.sleep(0.3)
    
    debug( ["GRAB", "LIFT"], [30, 20] )
    # Lift
    claw.close(90)
    claw.lift(180, 0.005)
    time.sleep(0.3)
    
    debug( ["GRAB", "VALIDATE"], [30, 20] )
    claw.read()
    debug( ["GRAB", f"{claw.spaces}"], [30, 20] )
    
    if claw.spaces[0 if insert == -1 else 1] != "": return True
    else:                                           return False

def dump(search_type: str) -> None:
    TIME_STEP = 0.2
    
    motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.3)
    motors.run(0, 0, 0.5)
    claw.read()
    
    # Determine angles to visit
    dump_type: str    
    if   search_type == "green": dump_type = "live"
    else:                        dump_type = "dead"
        
    angles = []
    if claw.spaces[0] == dump_type: angles.append(180)
    if claw.spaces[1] == dump_type: angles.append(0)

    # Dump
    claw.lift(90)
    
    for angle in angles:
        claw.close(angle)
        
        for _ in range(0, 2):
            claw.lift(80)
            time.sleep(TIME_STEP)
            claw.lift(90)
            time.sleep(TIME_STEP)

        time.sleep(0.3)
        
        evac_state.victims_saved += 1
        
    claw.close(90)
    claw.lift(180)
    time.sleep(1)
    
    claw.read()
    
evac_state = EvacuationState()
search = Search()
movement = Movement()

def main() -> None:    
    # # Warmup camera and TPU
    # for _ in range(0, 2): image = evac_camera.capture_image()
    # search.ai_dead(image, None)
    
    while True:
        claw.read()
        x, search_type = locate()
        
        route_success = route(x, search_type)
        if not route_success:
            # motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.5)
            continue
        
        # If it is a triangle
        if search_type in ["red", "green"]:
            # Reset and move closer again
            motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 1.5)
            route_success = route(x, search_type)
            if not route_success: continue
            
            dump(search_type)
            
            motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.5)
            motors.run( evac_state.fast_speed, -evac_state.fast_speed, 1)
        
        # Align only if we're picking up
        if search_type in ["live", "dead"]:
            align_success = align(10, 0.1)
            if not align_success: continue

            print("ALIGN SUCCESSFUL")
            
            grab_success = grab()
            if not grab_success:
                motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.7)
                continue