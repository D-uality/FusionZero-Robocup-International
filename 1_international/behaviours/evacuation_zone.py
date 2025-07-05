if __name__ == "__main__":
    import sys, pathlib
    root = pathlib.Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(root))

from core.shared_imports import time, np, cv2, Optional, math
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
        self.route_speed = 25
        self.grab_speed  = 30
        
        self.live_approach_distance     = 7
        self.dead_approach_distance     = 7
        self.triangle_approach_distance = 5
        self.align_distance = 4
        
        self.minimum_align_distance = 20
        
        self.silver_min = 130
        self.black_max  = 20
        
        self.gap_count = 3

        
class Search():
    def __init__(self):
        self.debug_live      = False
        self.debug_dead      = False
        self.debug_triangles = False

    def classic_live(self, image: np.ndarray, display_image: np.ndarray, last_x: Optional[int]) -> Optional[int]:
        THRESHOLD = 245
        KERNEL_SIZE = 15
        CROP_SIZE = 100
        
        working_image = image.copy()
        
        # Ensure image is not None
        if working_image is None or working_image.size == 0: return None
        
        x_lower = 0  # new: initialize crop offset
        
        # Crop according to last x
        if last_x is not None:
            x_lower = max(             0, last_x - CROP_SIZE)
            x_upper = min(image.shape[1], last_x + CROP_SIZE)
            
            if x_upper > x_lower: working_image = working_image[:, x_lower : x_upper]
            
        # Filter for spectral highlights
        spectral_highlights = cv2.inRange(working_image, (THRESHOLD, THRESHOLD, THRESHOLD), (255, 255, 255))
        spectral_highlights = cv2.dilate (spectral_highlights, np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8), iterations=1)

        contours, _ = cv2.findContours(spectral_highlights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None

        # Validate contours based on: area, y-position
        valid = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            
            if y + h/2 > 5 and y + h/2 < 80 and area < 3000 and area > 200:
                centre_x = x + w/2
                valid.append((contour, centre_x))
        if len(valid) == 0: return None

        # Find largest contour/closest contour
        best_contour = max(valid, key=lambda t: cv2.contourArea(t[0]))[0] if last_x is None else min(valid, key=lambda t: abs(t[1] - working_image.shape[1]/2))[0]
        x, _, w, _ = cv2.boundingRect(best_contour)

        if evac_state.X11:
            if x_lower > 0:
                adjusted_contour = best_contour.copy()
                adjusted_contour[:,:,0] += x_lower
                cv2.drawContours(display_image, [adjusted_contour], -1, (0, 255, 0), 2)
                
            else:
                cv2.drawContours(display_image, [best_contour], -1, (0, 255, 0), 2)
            
        if evac_state.X11 and self.debug_live:
            cv2.drawContours(working_image, [best_contour], -1, (0, 255, 0), 2)
            show(working_image, "live_working_image")
        
        # Find centre x
        centre_x = x + w/2 if last_x is None else x_lower + (x + w/2)
        return int(centre_x)
    
    def hough_dead(self, image: np.ndarray, display_image: np.ndarray, last_x: Optional[int]) -> Optional[int]:
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
        PARAMETER_1 =  55                              # Lower -> detect more circles   |  Higher -> detect less circles
        PARAMETER_2 =  25                              # Lower -> accepts more circles  |  Higher -> rejects more circles
        MIN_RADIUS =   20
        MAX_RADIUS =   230
                
        working_image = image.copy()
        
        # Ensure image is not None
        if working_image is None or working_image.size == 0: return None
        
        x_lower = 0  # new: initialize crop offset
        
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

        if self.debug_dead: show(grey_image, "pre_processing")
        
        t3 = time.perf_counter()
        # Hough Circle Transform
        circles = cv2.HoughCircles(grey_image, cv2.HOUGH_GRADIENT, DP, MIN_DISTANCE, param1=PARAMETER_1, param2=PARAMETER_2, minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)
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
        
        if len(valid) == 0: return None
        
        t5 = time.perf_counter()
        # Find the closest circle to the crop center, or the largest circle if last x is None
        if last_x is not None:
            crop_center = working_image.shape[1] / 2
            best_circle = min(valid, key=lambda circle: abs(circle[0] - crop_center))
            centre_x = best_circle[0] + x_lower
            
        else:
            best_circle = max(valid, key=lambda circle: circle[2])
            centre_x = best_circle[0] + x_lower
            
        # Display on image
        if evac_state.X11:
            _, y, r = best_circle
            cv2.circle(display_image, (centre_x, y), r, (0, 255, 0), 2)
            cv2.circle(display_image, (centre_x, y), 1, (0, 255, 0), 2)
            
            if self.debug_dead:
                cv2.circle(working_image, (x, y), r, (0, 255, 0), 2)
                cv2.circle(working_image, (x, y), 1, (0, 255, 0), 2)
                
                show(working_image, "dead_working_image")
                
        if self.debug_dead: print(f"Green: {(t1-t0)*1000:.1f}ms | Spectral: {(t2-t1)*1000:.1f}ms | Mask: {(t3-t2)*1000:.1f}ms | Hough: {(t4-t3)*1000:.1f}ms | Validation: {(t5-t4)*1000:.1f}ms | Total: {(t5-t0)*1000:.1f}ms")

        return int(centre_x)
    
    def triangle(self, image: np.ndarray, display_image: np.ndarray, triangle: str) -> tuple[Optional[int], Optional[int]]:
        KERNEL_SIZE = 3
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Select which mask to use
        if triangle == "red":
            mask_lower = cv2.inRange(hsv_image, (  0, 120, 0), ( 20, 255, 255))
            mask_upper = cv2.inRange(hsv_image, (160, 120, 0), (179, 255, 255))
            mask = cv2.bitwise_or(mask_lower, mask_upper)
        
        else:
            mask = cv2.inRange(hsv_image, (50, 120, 15), (85, 255, 255))
            
        mask = cv2.dilate(mask, np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8), iterations=1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None, None
        
        if self.debug_triangles: 
            for contour in contours: print(cv2.contourArea(contour))
        
        largest_contour = max(contours, key=cv2.contourArea)

        # If the largest contour is too small, return None
        
        contour_area = cv2.contourArea(largest_contour)
        if contour_area < 4500: return None, None

        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        if h > w or y + h/2 > evac_camera.height / 2: return None, None

        # Display debug information
        if evac_state.X11:
            cv2.drawContours(display_image, [largest_contour], -1, (0, 255, 0), 1)
            cv2.circle(display_image, (int(x + w / 2), int(y + h / 2)), 3, (255, 0, 0), 1)

        return int(x + w/2), contour_area

class Movement():
    def __init__(self):
        self.debug_path = True
        
        self.mode = "wall follow"
        
        self.touch_trigger = False
        self.touch_t0 = time.perf_counter()
        self.touch_count = 0
        
        self.spin_trigger = False
        self.spin_t0 = time.perf_counter()
            
    def route(self, kP: float, x: int, search_type: str) -> tuple[int]:
        MAX_VELOCITY = 40
        MIN_VELOCITY = -10
        
        error = evac_camera.width / 2 - x

        turn = kP * error
        scalar = 0.8 * turn if abs(error) < 10 else 1
        
        v1 = (evac_state.route_speed - turn * scalar if search_type in ["live", "dead"] else evac_state.fast_speed - turn)
        v2 = (evac_state.route_speed + turn * scalar if search_type in ["live", "dead"] else evac_state.fast_speed + turn)
        
        v1 = min(MAX_VELOCITY, max(v1, MIN_VELOCITY))
        v2 = min(MAX_VELOCITY, max(v2, MIN_VELOCITY))
        
        motors.run(v1, v2)
        
        return v1, v2
    
    def path(self) -> None:
        if   self.mode == "wall follow": self.wall_follow()
        elif self.mode == "spin cw":     self.__spin( evac_state.base_speed, -evac_state.base_speed)
        elif self.mode == "spin ccw":    self.__spin(-evac_state.base_speed,  evac_state.base_speed)
        
        return self.mode
    
    def __spin(self, v1: int, v2: int):
        MAX_TURN_TIME = 3
        
        if self.debug_path: debug( [f"{self.mode.upper()}", f"Time remaining: {MAX_TURN_TIME - time.perf_counter() + self.spin_t0:.2f}"], [35, 35])
        
        # Record start time once
        if not self.spin_trigger: self.spin_t0 = time.perf_counter()
        self.spin_trigger = True
        
        motors.run(v1, v2)
        
        # Reset when timer expires, swap modes
        if time.perf_counter() - self.spin_t0 > MAX_TURN_TIME:
            self.spin_trigger = False
            
            if self.mode == "spin cw": self.mode = "spin ccw"
            else:                      self.mode = "wall follow"
    
    def wall_follow(self, leaving=False) -> tuple[int]:
        KP = 1.2
        GAP_DISTANCE = 15                               # Threshold to classify for gap
        OFFSET = 3                                      # Wall following distance
        MAX_TURN = evac_state.base_speed * 0.5          # Max turning speed
        MAX_COUNT_TIME = 3
        PROJECTED_DISTANCE = 2
        
        touch_values = touch_sensors.read()
        distance = laser_sensors.read([0])[0]
        
        # Laser sensors failed
        if distance is None: return -1, -1
        
        error = distance - OFFSET
        raw_turn = KP * error
        
        if self.debug_path: debug( ["WALL FOLLOW", f"Count? {self.touch_trigger} {self.touch_count}", f"Time remaining: {MAX_COUNT_TIME - time.perf_counter() + self.touch_t0:.2f}"], [35, 35, 35])
        
        # Touch
        if sum(touch_values) != 2:
            # Record start time once
            if not self.touch_trigger: self.touch_t0 = time.perf_counter()
            
            # Activate countring trigger, and incremenet
            self.touch_trigger = True
            self.touch_count += 1
            
            if self.touch_count < 3:
                # Normal wall follow turn
                turn_time = 0.5 if distance < GAP_DISTANCE else 0.65
                motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.25)
                motors.run( evac_state.fast_speed, -evac_state.fast_speed, turn_time)
                
            else:
                # Reset counter, and change mode
                self.touch_count = 0
                self.touch_trigger = False
                self.mode = "spin cw"
            
        if time.perf_counter() - self.touch_t0 > MAX_COUNT_TIME:
            # Stop and reset counting when 3 seconds have pased
            self.touch_count = 0
            self.touch_trigger = False
            
        # Gap handling
        if distance > GAP_DISTANCE:
            # If leaving, turn into by uncapping max_turn and increasing raw_turn
            if leaving:
                raw_turn = raw_turn * 12
                MAX_TURN = evac_state.base_speed * 4
            
            # If searching, ignore and move forwards, early exit
            else:
                motors.run(evac_state.fast_speed, evac_state.fast_speed)
                return evac_state.fast_speed, evac_state.fast_speed
        
        # Project an artificial pivot point
        effective_turn = raw_turn * (1 + PROJECTED_DISTANCE / max(distance, OFFSET))
        effective_turn = min(max(effective_turn, -MAX_TURN), MAX_TURN)

        v1 = evac_state.base_speed - effective_turn
        v2 = evac_state.base_speed + effective_turn

        motors.run(v1, v2)

def analyse(image: np.ndarray, display_image: np.ndarray, search_type: str, last_x: Optional[int] = None) -> tuple[Optional[int], str]:      
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
        live_x = search.classic_live(image, display_image, last_x)
        if live_x is not None: return live_x, "live"

    if search_type in ["dead", "green", "default"] and dead_enable:
        dead_x = search.hough_dead(image, display_image, last_x)
        if dead_x is not None: return dead_x, "dead"
    
    if search_type in ["green", "default"] and green_enable:
        x, _ = search.triangle(image, display_image, "green")
        if x is not None: return x, "green"
    
    if search_type in ["red", "default"] and red_enable:
        x, _  = search.triangle(image, display_image, "red")
        if x is not None: return x, "red"
    
    return None, search_type

def locate(black_count: int, silver_count: int, search_type: str = "default") -> tuple[int, str, int, int]:
    while True:
        colour_values = colour_sensors.read()
        image = evac_camera.capture()
        display_image = image.copy()
        
        # Validate gaps
        black_count, silver_count = validate_exit(colour_values, black_count, silver_count)
        if black_count > evac_state.gap_count or silver_count > evac_state.gap_count:
            motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.5)
            motors.run( evac_state.fast_speed, -evac_state.fast_speed, 1)
        
        # Exit if target found
        x, search_type = analyse(image, display_image, search_type)
        if x is not None: return x, search_type, black_count, silver_count
        
        v1, v2 = movement.path()
        
        if evac_state.X11:   show(display_image, name="display", display=True)
        if evac_state.debug: debug( [f"LOCATING", f"{v1:.2f} {v2:.2f}", f"search: {search_type}", f"victims: {evac_state.victims_saved}", f"claw: {claw.spaces}", f"{black_count, silver_count}"], [30, 20, 20, 20, 20, 20] )

def route(last_x: int, search_type: str, black_count: int, silver_count: int) -> bool:
    while True:
        # Take measurements
        distance = laser_sensors.read([1])[0]
        colour_values = colour_sensors.read()
        image = evac_camera.capture()
        display_image = image.copy()
        
        x, search_type = analyse(image, display_image, search_type, last_x)
        silver_count, black_count = validate_exit(colour_values, black_count, silver_count)
        
        # Error handling
        if distance is None:                    continue
        if        x is None:                    return False
        
        if black_count > evac_state.gap_count or silver_count > evac_state.gap_count:
            motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.5)
            motors.run( evac_state.fast_speed, -evac_state.fast_speed, 1)
            return False
        
        # Exit conditions
        if   search_type in ["live"]         and distance < evac_state.live_approach_distance:     return True
        elif search_type in ["dead"]         and distance < evac_state.dead_approach_distance:     return True
        elif search_type in ["red", "green"] and distance < evac_state.triangle_approach_distance: break
        
        # Route with kP        
        v1, v2 = movement.route(0.25, x, search_type)
        
        last_x = x
        
        if evac_state.X11:   show(display_image, "display")
        if evac_state.debug: debug( ["ROUTING", f"{search_type}", f"Distance: {distance:.2f}", f"x: {x} {last_x}", f"{v1:.2f} {v2:.2f}", f"{black_count, silver_count}"], [30, 20, 20, 20, 20] )

    while True:
        if sum(touch_sensors.read()) == 0: break
        motors.run(evac_state.fast_speed, evac_state.fast_speed)
    
    return True

def grab() -> bool:    
    if "" not in claw.spaces: return False
    
    # Left: -1, Right: 1
    insert = -1 if claw.spaces[0] == "" else 1
    
    debug( ["GRAB", "SETUP"], [30, 20] )
    # Setup
    motors.run(         -evac_state.grab_speed,         -evac_state.grab_speed, 0.3)
    motors.run(-evac_state.grab_speed * insert, evac_state.grab_speed * insert, 0.2)
    motors.run(0, 0, 0.3)
    
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

def align_line(align_type: str) -> None:
    if align_type == "silver":   

        motors.run( evac_state.base_speed,       evac_state.base_speed,   0.7)
        motors.run(-evac_state.base_speed * 0.3, -evac_state.base_speed * 0.3)
        left_silver, right_silver = False, False
        
        # Move backwards till 1 finds silver
        while not left_silver and not right_silver:
            colour_values = colour_sensors.read()

            if colour_values[0] > evac_state.silver_min or colour_values[1] > evac_state.silver_min:
                left_silver = True
                print("Found left Silver")

            elif colour_values[3] > evac_state.silver_min or colour_values[4] > evac_state.silver_min:
                right_silver = True
                print("Found right silver")
        
        # Moving forwards slowly
        motors.run(evac_state.base_speed * 0.5, evac_state.base_speed * 0.5, 0.4)

        # Account for angled entry
        if(left_silver): 
            motors.run_until(7, -evac_state.base_speed * 0.5, colour_sensors.read, 4, ">=", evac_state.silver_min, "Right")
        
        # Repeat finding silver
        for i in range(3):
            forwards_speed = evac_state.base_speed * 0.5
            other_speed = 8
            
            motors.run      ( 0, 0, 0.2)
            motors.run_until(-forwards_speed,     other_speed, colour_sensors.read, 0, ">=", evac_state.silver_min, "LEFT SILVER")
            
            motors.run      ( 0, 0, 0.2)
            motors.run_until(    other_speed, -forwards_speed, colour_sensors.read, 4, ">=", evac_state.silver_min, "RIGHT SILVER")
            
            motors.run      (0, 0, 0.2)
            motors.run_until( forwards_speed,     other_speed, colour_sensors.read, 0, "<=", evac_state.silver_min, "LEFT WHITE")
            
            motors.run      ( 0, 0, 0.2)
            motors.run_until(    other_speed,  forwards_speed, colour_sensors.read, 4, "<=", evac_state.silver_min, "RIGHT WHITE")
                    
    elif align_type == "black":
        motors.run(-evac_state.base_speed*0.7, -evac_state.base_speed*0.7, 0.7)
        motors.run(evac_state.base_speed * 0.5, evac_state.base_speed * 0.5)
        left_black, right_black = None, None
        while left_black is None and right_black is None:
            colour_values = colour_sensors.read()

            if colour_values[0] < 40 or colour_values[1] < 40:
                left_black = True
                print("Found left black")
            elif colour_values[3] < 40 or colour_values[4] < 40:
                right_black = True
                print("Found right black")

        if(left_black): 
            motors.run_until(-5, evac_state.base_speed * 0.5, colour_sensors.read, 4, "<=", 30, "Right")
            
        for i in range(3):
            motors.run(0, 0, 0.2)
            motors.run_until(-evac_state.base_speed * 0.5, -5, colour_sensors.read, 0, ">=", 60, "Left")
            motors.run(0, 0, 0.2)
            motors.run_until(-5, -evac_state.base_speed * 0.5, colour_sensors.read, 4, ">=", 60, "Right")
            motors.run(0, 0, 0.2)
            motors.run_until(evac_state.base_speed * 0.5, -5, colour_sensors.read, 0, "<=", 30, "Left")
            motors.run(0, 0, 0.2)
            motors.run_until(-5, evac_state.base_speed * 0.5, colour_sensors.read, 4, "<=", 30, "Right")

        for i in range(0, int(evac_state.base_speed * 0.7)):
            motors.run(-evac_state.base_speed*0.7+i, -evac_state.base_speed*0.7+i, 0.03)

def validate_exit(colour_values: list[int], black_count: int, silver_count: int) -> bool:
    valid_values = [colour_values[0], colour_values[1], colour_values[3], colour_values[4]]
    silver_values = [1 if value >= evac_state.silver_min else 0 for value in valid_values]
    black_values  = [1 if value <=  evac_state.black_max else 0 for value in valid_values]
    
    silver_count = silver_count + sum(silver_values) if sum(silver_values) >= 1 else 0
    black_count  = black_count  + sum(black_values)  if sum(black_values)  >= 1 else 0
    
    return silver_count, black_count

def leave():
    silver_count = black_count = 0
    
    if evac_state.victims_saved != 3:
        motors.run_until(evac_state.fast_speed, evac_state.fast_speed, touch_sensors.read, 0, "==", 0, "FORWARDS LEFT")
        motors.run_until(evac_state.fast_speed, evac_state.fast_speed, touch_sensors.read, 1, "==", 0, "FORWARDS RIGHT")
        motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.3)
        motors.run(evac_state.fast_speed, -evac_state.fast_speed, 1)    
    
    while True:
        movement.wall_follow(leaving=True)
        colour_values = colour_sensors.read()
        silver_count, black_count = validate_exit(colour_values, black_count, silver_count)

        if black_count >= 5:
            debug(["EXITING", "FOUND EXIT!"], [24, 30])
            align_line("black")
            break
        
        elif silver_count >= 5:
            debug(["EXITING", "FOUND SILVER"], [24, 30])
            
            motors.run(0, 0, 0.3)
            align_line("silver")
            motors.run(-evac_state.base_speed, -evac_state.base_speed, 1.7)
            motors.run(0, 0, 0.3)
            motors.run(evac_state.base_speed, -evac_state.base_speed, 2.2)
            motors.run(0, 0, 0.3)

            while True:
                distance = laser_sensors.read([0])[0]
                touch_values = touch_sensors.read()
                
                debug(["MOVING TILL WALL", f"{distance}", f"{touch_values}"], [24, 10, 10])
                motors.run(22, 22)

                if distance <= 20:
                    break
                
                if sum(touch_values) != 2:
                    motors.run(evac_state.base_speed, evac_state.base_speed, 1)
                    motors.run(-evac_state.base_speed, -evac_state.base_speed, 0.5)
                    motors.run(evac_state.fast_speed, -evac_state.fast_speed, 1)
                    motors.run_until(evac_state.base_speed,  -evac_state.base_speed, laser_sensors.read, 0, "<=", 10)

                    break

#---------------------------------------------------------------------------------------------------------------------------------------------

evac_state = EvacuationState()
search = Search()
movement = Movement()

def main() -> None:    
    # Warmup camera
    for _ in range(0, 2): _ = evac_camera.capture()
    led.off()

    black_count = silver_count = 0
    
    while True:
        if evac_state.victims_saved == 3: break
        
        claw.read()
        x, search_type, black_count, silver_count = locate(black_count, silver_count)
        
        route_success = route(x, search_type, black_count, silver_count)
        if not route_success: continue
                
        # If it is a triangle
        if search_type in ["red", "green"]:
            # Reset and move closer again
            motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 2)
            x, search_type, black_count, silver_count = locate(search_type)
            
            route_success = route(x, search_type, black_count, silver_count)
            if not route_success: continue
            
            dump(search_type)
            
            motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.2)
            motors.run( evac_state.fast_speed, -evac_state.fast_speed, 1)
        
        # Align only if we're picking up
        if search_type in ["live", "dead"]:
            
            grab_success = grab()
            if not grab_success:
                motors.run(-evac_state.fast_speed, -evac_state.fast_speed, 0.7)
                continue
            
    leave()

#---------------------------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    # evac_state.victims_saved = 3
    
    main()