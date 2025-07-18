if __name__ == "__main__":
    import sys, pathlib
    root = pathlib.Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(root))

from core.shared_imports import time, np, cv2, Optional, randint, math
from hardware.robot import *
from core.utilities import *
from core.listener import listener


start_display()

##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################

class EvacuationState:
    def __init__(self):
        self.DISPLAY    = True
        self.DEBUG_MAIN = True
        
        self.victim_count = 0
        
        # Speeds        
        if user_at_host == "frederick@raspberrypi":
            self.SPEED_BASE  = 35
            self.SPEED_FAST  = 45
            self.SPEED_ROUTE = 25
            self.SPEED_GRAB  = 30
        else:
            self.SPEED_BASE  = 25
            self.SPEED_FAST  = 40
            self.SPEED_ROUTE = 18
            self.SPEED_GRAB  = 30
            
        # Locate settings
        self.DEBUG_LOCATE = False
        
        # Analyse settings
        self.DEBUG_ANALYSE = False
        
        # Route settings
        self.DEBUG_ROUTE = True
        self.ROUTE_APPROACH_VICTIM_DISTANCE = 7
        self.ROUTE_APPROACH_TRIANGLE_DISTANCE = 7
        self.last_victim_time = 0
        self.MIN_VICTIM_TRIANGLE_SWITCH_TIME = 1
        
        # Gap handling
        self.SILVER_MIN       = 185
        self.BLACK_MAX        = 40
        self.GAP_BLACK_COUNT  = 0
        self.GAP_SILVER_COUNT = 0

class Search():
    def __init__(self, evac_state: EvacuationState, evac_camera: EvacuationCamera):
        # Debug
        self.DISPLAY: bool = evac_state.DISPLAY
        
        self.DEBUG_LIVE: bool       = False
        self.DEBUG_DEAD: bool       = True
        self.DEBUG_TRIANGLES: bool  = False
        
        self.TIMING_LIVE: bool      = False
        self.TIMING_DEAD: bool      = True
        self.TIMING_TRIANGLES: bool = False
        
        # General settings
        self.CROP_SIZE = 200
        self.IMG_CENTRE_X = evac_camera.width / 2
        
        # Live settings
        self.LIVE_THRESHOLD = 245
        self.LIVE_ERODE_KERNAL = np.ones((1, 1), np.uint8)
        self.LIVE_DILATE_KERNAL = np.ones((25, 25), np.uint8)
        
        self.LIVE_UNDILATE_MIN_AREA = 100
        self.LIVE_UNDILATE_MIN_COUNT = 4
        self.LIVE_UNDILATE_MIN_COUNT_MIN_AREA = 3
        
        self.LIVE_MIN_AREA = 150
        self.LIVE_MAX_AREA = 1000000
        
        self.LIVE_MIN_Y = 25
        self.LIVE_MAX_Y = 60
        
        # Dead settings
        self.DEAD_GREEN_KERNAL = np.ones((7,  7), np.uint8)
        self.DEAD_WHITE_KERNAL = np.ones((9,  9), np.uint8)
        self.DEAD_BLACK_KERNAL = np.ones((25, 25), np.uint8)
        
        self.DEAD_WHITE_THRESHOLD = (160, 160, 160)
        self.DEAD_BLACK_THRESHOLD = (60, 60, 60)
        
        self.DEAD_MIN_BLACK_AREA = 300
        self.DEAD_MIN_Y = 40
        self.DEAD_RADIUS_Y_MIN = -50
        
        self.HOUGH_DP =           1                               # "Resolution" of the accumulator
        self.HOUGH_MIN_DISTANCE = 200
        self.HOUGH_PARAMETER_1 =  50                              # Lower -> detect more circles   |  Higher -> detect less circles
        self.HOUGH_PARAMETER_2 =  30                              # Lower -> accepts more circles  |  Higher -> rejects more circles
        self.HOUGH_MIN_RADIUS =   5
        self.HOUGH_MAX_RADIUS =   150
        
        # Triangle settings
        self.TRIANGLE_TOP_CROP = 15
        
        self.TRIANGLE_GREEN_HSV = ((50, 120, 15), (85, 255, 255))
        self.TRIANGLE_ERODE_KERNAL = np.ones((5, 5), np.uint8)
        self.TRIANGLE_DILATE_KERNAL = np.ones((5, 5), np.uint8)
        
        self.TRIANGLE_MIN_Y = 100 - self.TRIANGLE_TOP_CROP
        self.TRIANGLE_MIN_AREA = 600
        self.TRIANGLE_MIN_TRIANGULARITY = 0.7
        
        
    def live(self, image: np.ndarray, display_image: np.ndarray, last_x: Optional[int]) -> Optional[int]:
        if image is None or image.size == 0: return None
        
        # Crop according to last x
        if last_x is not None:
            x_lower = max(0, last_x - self.CROP_SIZE)
            x_upper = min(image.shape[1], last_x + self.CROP_SIZE)
            working_image = image[:, x_lower:x_upper]
            
        else:
            working_image = image.copy()
            x_lower = 0
        
        cropped_image = working_image[self.LIVE_MIN_Y:self.LIVE_MAX_Y, :]
        spectral_highlights = cv2.inRange(cropped_image, (self.LIVE_THRESHOLD, self.LIVE_THRESHOLD, self.LIVE_THRESHOLD), (255, 255, 255))
        original_highlights = spectral_highlights.copy()
        
        if self.DEBUG_LIVE and self.DISPLAY:
            cv2.line(display_image, (0, self.LIVE_MIN_Y), (100, self.LIVE_MIN_Y), (255, 0, 0), 1)
            cv2.line(display_image, (0, self.LIVE_MAX_Y), (100, self.LIVE_MAX_Y), (255, 0, 0), 1)
                    
        spectral_highlights = cv2.dilate(spectral_highlights, self.LIVE_DILATE_KERNAL, iterations=1)        
        
        # Find all contours within the dilated spectral_highlights mask
        dilated_contours, _ = cv2.findContours(spectral_highlights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Begin dilated_contours validation
        valid_contours = []
        
        for contour in dilated_contours:
            # Create a mask based on this contour
            connectivity_mask = np.zeros_like(spectral_highlights)
            cv2.drawContours(connectivity_mask, [contour], -1, 255, thickness=cv2.FILLED)
        
            # Mask the original mask with the dilated mask
            feature_mask = cv2.bitwise_and(original_highlights, original_highlights, mask=connectivity_mask)
                        
            # Find the sub_contours within this region
            sub_contours, _ = cv2.findContours(feature_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Perform validation
            total_area = sum(cv2.contourArea(c) for c in sub_contours)
            
            if self.DEBUG_LIVE: print(f"\tArea check: {total_area} > {self.LIVE_UNDILATE_MIN_AREA}    |    {len(sub_contours)} > {self.LIVE_UNDILATE_MIN_COUNT}")
            
            if total_area > self.LIVE_UNDILATE_MIN_AREA or (len(sub_contours) > self.LIVE_UNDILATE_MIN_COUNT and total_area > self.LIVE_UNDILATE_MIN_COUNT_MIN_AREA):
                valid_contours.append(contour)
                
            if self.DEBUG_LIVE and self.DISPLAY:
                for contour in sub_contours: contour[:, 0, 1] += self.LIVE_MIN_Y
                cv2.drawContours(display_image, sub_contours, -1, (0, 255, 255), 2)
        
        if len(valid_contours) == 0: return None
        
        if self.DEBUG_LIVE and self.DISPLAY:
            for contour in valid_contours: contour[:, 0, 1] += self.LIVE_MIN_Y
            cv2.drawContours(display_image, valid_contours, -1, (0, 255, 0), 2)
        
        valid_contours.sort(key=cv2.contourArea, reverse=True)
        largest_contour = valid_contours[0]
        
        x, _, w, _ = cv2.boundingRect(largest_contour)
        centre_x = x + w // 2
        
        return centre_x + x_lower
    
    def dead(self, image: np.ndarray, display_image: np.ndarray, last_x: Optional[int]) -> Optional[int]:
        if image is None or image.size == 0: return None
        if self.TIMING_DEAD: start_time = time.perf_counter()
            
        # Crop according to last x
        if last_x is not None:
            x_lower = max(0, last_x - self.CROP_SIZE)
            x_upper = min(image.shape[1], last_x + self.CROP_SIZE)
            working_image = image[:, x_lower:x_upper]
            
        else:
            working_image = image.copy()
            x_lower = 0
            
        if self.TIMING_DEAD: green_done_time = time.perf_counter()
                
        # Normalise white
        white = cv2.inRange(working_image, self.DEAD_WHITE_THRESHOLD, (255, 255, 255))
        white = cv2.erode(white, self.DEAD_WHITE_KERNAL, iterations=1)
        white = cv2.dilate(white, self.DEAD_WHITE_KERNAL, iterations=1)
        working_image[white > 0] = [160, 160, 160]
        if self.TIMING_DEAD: white_done_time = time.perf_counter()
        
        # crop based on black mask
        black_mask  = cv2.inRange(working_image, (0, 0, 0), self.DEAD_BLACK_THRESHOLD)
        if self.DEBUG_DEAD: show(black_mask, name="black mask", display=True)
        
        # Remove small black contours from mask
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_mask = np.zeros_like(black_mask)
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if self.DEBUG_DEAD: print(f"{contour_area}", end="  |  ")
            
            if contour_area >= self.DEAD_MIN_BLACK_AREA:
                cv2.fillPoly(filtered_mask, [contour], 255)
        black_mask = filtered_mask
        
        black_mask  = cv2.dilate(black_mask, self.DEAD_BLACK_KERNAL, iterations=1)
        
        # Search for only black parts of the image
        grey_image = cv2.cvtColor(working_image, cv2.COLOR_BGR2GRAY)
        grey_image = cv2.bitwise_and(grey_image, black_mask)

        if self.DEBUG_DEAD: show(grey_image, "pre_processing")
        if self.TIMING_DEAD: mask_done_time = time.perf_counter()
        
        # Hough Circle Transform
        circles = cv2.HoughCircles(grey_image, cv2.HOUGH_GRADIENT, self.HOUGH_DP, self.HOUGH_MIN_DISTANCE, param1=self.HOUGH_PARAMETER_1, param2=self.HOUGH_PARAMETER_2, minRadius=self.HOUGH_MIN_RADIUS, maxRadius=self.HOUGH_MAX_RADIUS)
        if circles is None: return None
        if self.TIMING_DEAD: hough_done_time = time.perf_counter()

        if self.DEBUG_DEAD: print(f"Circles before: {circles}")
        # Process circles
        circles = np.round(circles[0, :]).astype("int")
        
        # Validate circles based on size
        valid = []
        
        for (x, y, r) in circles:
            x = max(0, min(x, working_image.shape[1] - 1))
            y = max(0, min(y, working_image.shape[0] - 1))
            
            if self.DEBUG_DEAD: print(f"{y} {self.DEAD_MIN_Y}    |    {y-r} {self.DEAD_RADIUS_Y_MIN}")
            
            if y > self.DEAD_MIN_Y and y - r > self.DEAD_RADIUS_Y_MIN:
                valid.append((x, y, r))
        
        if self.DEBUG_DEAD: print(f"Valid: {valid}")
        if len(valid) == 0: return None
        
        # Find the closest circle to the crop center, or the largest circle if last x is None
        if last_x is not None:
            crop_center = working_image.shape[1] / 2
            best_circle = min(valid, key=lambda circle: abs(circle[0] - crop_center))
            centre_x = best_circle[0] + x_lower
        else:
            best_circle = max(valid, key=lambda circle: circle[2])
            centre_x = best_circle[0] + x_lower
            
        if self.TIMING_DEAD: validation_done_time = time.perf_counter()
            
        # Display on image
        if evac_state.DISPLAY:
            _, y, r = best_circle
            cv2.circle(display_image, (centre_x, y), r, (0, 255, 0), 2)
            cv2.circle(display_image, (centre_x, y), 1, (0, 255, 0), 2)
        
        if self.TIMING_DEAD:
            print(f"Green: {(green_done_time-start_time)*1000:.1f}ms | Spectral: {(white_done_time-green_done_time)*1000:.1f}ms | Mask: {(mask_done_time-white_done_time)*1000:.1f}ms | Hough: {(hough_done_time-mask_done_time)*1000:.1f}ms | Validation: {(validation_done_time-hough_done_time)*1000:.1f}ms | Total: {(validation_done_time-start_time)*1000:.1f}ms")

        return int(centre_x)

    def triangle(self, image: np.ndarray, display_image: np.ndarray, triangle: str) -> tuple[Optional[int], Optional[int]]:
        if image is None or image.size == 0: return None, None
        if self.TIMING_TRIANGLES: start_time = time.perf_counter()

        # Crop the image to remove the top 30 pixels
        image = image[self.TRIANGLE_TOP_CROP:, :]
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.TIMING_TRIANGLES: hsv_time = time.perf_counter()
        
        # Select which mask to use
        if triangle == "red":
            mask_lower = cv2.inRange(hsv_image, (  0, 120, 0), ( 20, 255, 255))
            mask_upper = cv2.inRange(hsv_image, (160, 120, 0), (179, 255, 255))
            mask = cv2.bitwise_or(mask_lower, mask_upper)
            
            if self.DISPLAY and self.DEBUG_TRIANGLES: show(mask, name="red mask", display=True)
        
        else:
            mask = cv2.inRange(hsv_image, self.TRIANGLE_GREEN_HSV[0], self.TRIANGLE_GREEN_HSV[1])
            
            if self.DISPLAY and self.DEBUG_TRIANGLES: show(mask, name="greens mask", display=True)
        
        if self.TIMING_TRIANGLES: mask_time = time.perf_counter()
        
        mask =  cv2.erode(mask, self.TRIANGLE_ERODE_KERNAL,  iterations=1)
        mask = cv2.dilate(mask, self.TRIANGLE_DILATE_KERNAL, iterations=1)
        
        if self.TIMING_TRIANGLES: dilate_time = time.perf_counter()
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            if self.TIMING_TRIANGLES:
                total_time = time.perf_counter()
                print(f"triangle() | HSV: {(hsv_time-start_time)*1000:.1f}ms | Mask: {(mask_time-hsv_time)*1000:.1f}ms | Dilate: {(dilate_time-mask_time)*1000:.1f}ms | Total: {(total_time-start_time)*1000:.1f}ms | Status: no_contours")
            return None, None
        
        if self.TIMING_TRIANGLES: contours_time = time.perf_counter()
        if self.DEBUG_TRIANGLES:
            for contour in contours: print(f"{cv2.contourArea(contour)}", end="  |  ")
        
        largest_contour = max(contours, key=cv2.contourArea)
        contour_area = cv2.contourArea(largest_contour)
        rectangularity = self._rectangularity_score(largest_contour)
        
        if self.DEBUG_TRIANGLES: print(f"Rectangle Score: {rectangularity}")
                
        if contour_area < self.TRIANGLE_MIN_AREA or rectangularity < self.TRIANGLE_MIN_TRIANGULARITY:
            if self.TIMING_TRIANGLES:
                total_time = time.perf_counter()
                print(f"triangle() | HSV: {(hsv_time-start_time)*1000:.1f}ms | Mask: {(mask_time-hsv_time)*1000:.1f}ms | Dilate: {(dilate_time-mask_time)*1000:.1f}ms | Contours: {(contours_time-dilate_time)*1000:.1f}ms | Total: {(total_time-start_time)*1000:.1f}ms | Status: too_small")
            return None, None
        
        if self.TIMING_TRIANGLES: area_time = time.perf_counter()
        
        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        if self.DEBUG_TRIANGLES: print(f"h>w: {h} {w}  |  y + h/2: {y + h/2} {evac_camera.height/2}")
        
        if h > w or y + h/2 > self.TRIANGLE_MIN_Y:
            if self.TIMING_TRIANGLES:
                total_time = time.perf_counter()
                print(f"triangle() | HSV: {(hsv_time-start_time)*1000:.1f}ms | Mask: {(mask_time-hsv_time)*1000:.1f}ms | Dilate: {(dilate_time-mask_time)*1000:.1f}ms | Contours: {(contours_time-dilate_time)*1000:.1f}ms | Area: {(area_time-contours_time)*1000:.1f}ms | Total: {(total_time-start_time)*1000:.1f}ms | Status: invalid_shape")
            return None, None
        
        if self.TIMING_TRIANGLES: bounds_time = time.perf_counter()
        
        # Display debug information
        if self.DISPLAY:
            cv2.drawContours(display_image, [largest_contour], -1, (0, 255, 0), 1)
            cv2.circle(display_image, (int(x + w / 2), int(y + h / 2)), 3, (255, 0, 0), 1)
        
        if self.TIMING_TRIANGLES:
            total_time = time.perf_counter()
            print(f"triangle() | HSV: {(hsv_time-start_time)*1000:.1f}ms | Mask: {(mask_time-hsv_time)*1000:.1f}ms | Dilate: {(dilate_time-mask_time)*1000:.1f}ms | Contours: {(contours_time-dilate_time)*1000:.1f}ms | Area: {(area_time-contours_time)*1000:.1f}ms | Bounds: {(bounds_time-area_time)*1000:.1f}ms | Display: {(total_time-bounds_time)*1000:.1f}ms | Total: {(total_time-start_time)*1000:.1f}ms | Status: success")

        return int(x + w/2), contour_area
    
    def _rectangularity_score(self, contour: np.ndarray) -> float:
        # 1) area-fit
        rect = cv2.minAreaRect(contour)
        box  = cv2.boxPoints(rect)
        
        boxArea      = cv2.contourArea(box)
        contourArea  = cv2.contourArea(contour)
        
        if boxArea == 0: return 0.0
        areaScore = contourArea / boxArea                        # ≤ 1

        # 2) vertex-count fit
        peri   = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        vertexScore = 1 - abs(len(approx) - 4) / 4               # 1 if 4 vertices

        # 3) angle fit (only if we really have 4 corners)
        if len(approx) == 4:
            pts = approx.reshape(-1, 2)
            angles = []
            for i in range(4):
                p0, p1, p2 = pts[i], pts[(i + 1) % 4], pts[(i + 2) % 4]
                v1, v2 = p0 - p1, p2 - p1
                
                cosang = np.clip(
                    np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)),
                    -1.0, 1.0
                )
                
                angle = math.degrees(math.acos(cosang))
                angles.append(angle)
            angleScore = sum(1 - abs(90 - a) / 90 for a in angles) / 4  # 1 if 90°
        else:
            angleScore = 0.0

        # combine
        score = max(0.0, min(1.0, areaScore * vertexScore * angleScore))
        return score
    
class Movement():
    def __init__(self, evac_state: EvacuationState, evac_camera: EvacuationCamera, touch_sensors: TouchSensors, colour_sensors: ColourSensors, motors: Motors):
        # Hardware
        self.touch_sensors = touch_sensors
        self.colour_sensors = colour_sensors
        self.motors = motors
        
        # Speed settings
        self.SPEED_BASE  = evac_state.SPEED_BASE 
        self.SPEED_FAST  = evac_state.SPEED_FAST 
        self.SPEED_ROUTE = evac_state.SPEED_ROUTE
        
        # Routing settings
        self.IMG_CENTRE_X = evac_camera.width / 2
        self.ROUTE_MAX_VELOCITY = 40
        self.ROUTE_MIN_VELOCITY = -10
        
        # Gap settings
        self.SILVER_MIN = 120
        self.BLACK_MAX = 30
        
        # Pathing settings
        self.spin_start_time = time.perf_counter()
        self.spin_timer_trigger = False
        
        # Wall follow settings
        self.WF_KP = 1.2
        self.WF_GAP_DISTANCE = 15
        self.WF_OFFSET = evac_state.SPEED_BASE * 0.5
        self.WF_PROJECTED_DISTANCE = 2
        self.integral=0

    def route(self, kP: float, x: int, search_type: str) -> None:
        error = evac_camera.width / 2 - x

        turn = kP * error
        scalar = 0.8 if abs(error) < 10 else 1
        
        v1 = ((self.SPEED_ROUTE - turn) * scalar if search_type in ["live", "dead"] else self.SPEED_FAST - turn)
        v2 = ((self.SPEED_ROUTE + turn) * scalar if search_type in ["live", "dead"] else self.SPEED_FAST + turn)
        
        v1 = min(self.ROUTE_MAX_VELOCITY, max(v1, self.ROUTE_MIN_VELOCITY))
        v2 = min(self.ROUTE_MAX_VELOCITY, max(v2, self.ROUTE_MIN_VELOCITY))
        
        motors.run(v1, v2)
        
    def wall_follow(self) -> None:
        KP = 1.2
        GAP_DISTANCE = 14                             # Threshold to classify for gap
        OFFSET = 3                                      # Wall following distance
        MAX_TURN = evac_state.SPEED_BASE * 0.5          # Max turning speed
        
        touch_values = touch_sensors.read()
        distance = laser_sensors.read([0])[0]
        
        # Laser sensors failed
        if distance is None: distance = 255
        
        error = distance - OFFSET
        if error > 18: self.integral += error
        if self.integral > 10000: self.integral = 10000
        print(self.integral)
        raw_turn = KP * error
        
        # Touch
        if sum(touch_values) != 2:
            turn_time = 0.5 if distance < GAP_DISTANCE else 0.65

            motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.25)
            motors.run( evac_state.SPEED_FAST, -evac_state.SPEED_FAST, turn_time)

        # Turn into gaps if leaving, uncap max_turn and increase the raw_turn
        if distance > GAP_DISTANCE:
            raw_turn = raw_turn * 1000
            MAX_TURN = evac_state.SPEED_BASE * 5000
            if self.integral > 9000:
                touch_count = 0
    
                while listener.mode.value != 0:
                    touch_values = touch_sensors.read()
                    
                    if sum(touch_values) == 0:
                        touch_count += 1
                        
                    elif sum(touch_values) == 2:
                        touch_count = 0
                        motors.run(35, 35)
                        
                    elif touch_values[0] == 0:
                        motors.run(-12, 35)
                        touch_count = 0
                        
                    elif touch_values[1] == 0:
                        motors.run(35, -12)
                        touch_count = 0
                        
                    if touch_count > 5:
                        break
                    
                self.integral = 0

                motors.run(0, 0)
                
                return True
        else:
            self.integral -= 1000
            if self.integral < 0: self.integral = 0
            
        
        # Project an artificial pivot point
        effective_turn = raw_turn * (1 + 2 / max(distance, OFFSET))
        effective_turn = min(max(effective_turn, -MAX_TURN), MAX_TURN)

        v1 = evac_state.SPEED_BASE - effective_turn
        v2 = evac_state.SPEED_BASE + effective_turn

        if distance > GAP_DISTANCE: v1, v2 = -100, 100
        motors.run(v1, v2) 
        if distance > GAP_DISTANCE:
            time.sleep(0.3)

##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################

def analyse(image: np.ndarray, display_image: np.ndarray, search_type: str, last_x: Optional[int] = None) -> tuple[Optional[int], str]:      
    live_enable  = False
    dead_enable  = False
    green_enable = False
    red_enable   = False
    
    claw.read()
    live_count  = claw.spaces.count("live")
    dead_count  = claw.spaces.count("dead")
    empty_count = 2 - live_count - dead_count
        
    if evac_state.victim_count + live_count < 2 and empty_count != 0: live_enable  = True
    if dead_count < 1 and empty_count != 0:                           dead_enable  = True
    if live_count > 0:                                                green_enable = True
    if dead_count == 1 and evac_state.victim_count == 2:              red_enable   = True
    
    if evac_state.DEBUG_ANALYSE: print(live_enable, dead_enable, green_enable, red_enable)
    
    if search_type in ["live", "green", "default"] and live_enable:
        live_x = op_search.live(image, display_image, last_x)
        if live_x is not None: return live_x, "live"

    if search_type in ["dead", "green", "default"] and dead_enable:
        dead_x = op_search.dead(image, display_image, last_x)
        if dead_x is not None: return dead_x, "dead"
    
    if search_type in ["green", "default"] and green_enable:
        x, _ = op_search.triangle(image, display_image, "green")
        if x is not None: return x, "green"
    
    if search_type in ["red", "default"] and red_enable:
        x, _  = op_search.triangle(image, display_image, "red")
        if x is not None: return x, "red"
        
    return None, search_type

def locate(black_count: int, silver_count: int) -> tuple[int, int, int, str]:
    mode = "forwards" 
    spinning_start_time = time.perf_counter()
    forwards_start_time = time.perf_counter()
    
    spinning_time = randint(40, 80) / 10 # 30, 50 on old (fast)
    forwards_time = randint(30, 50) / 10 # 30, 50 on old (fast)
    
    spinning_direction = 1
    
    while listener.mode.value != 0:
        print(mode)
        
        # Read sensor stack
        if evac_state.DEBUG_LOCATE: print("\tReading sensor stack")
        silver_value = silver_sensor.read()
        touch_values = touch_sensors.read()
        gyro_value = gyroscope.read()
        
        if gyro_value is not None:
            _, pitch, _ = gyro_value
        
        black_count, silver_count = validate_gap(silver_value, black_count, silver_count)
        
        # clear conditions
        touch_activated = sum(touch_values) != 2
        forwards_expire = time.perf_counter() - forwards_start_time > forwards_time
        gap_found       = black_count > evac_state.GAP_BLACK_COUNT or silver_count > evac_state.GAP_SILVER_COUNT
        out_of_bounds   = pitch > 10 if pitch is not None else False
        
        print(touch_activated, forwards_expire, gap_found, out_of_bounds)
        
        if evac_state.DEBUG_LOCATE: print(f"\tConditions | Touch: {touch_activated} Expire: {forwards_expire} Gap: {gap_found}")
    
        if gap_found: black_count = silver_count = 0
        if (touch_activated or forwards_expire or gap_found or out_of_bounds) and mode == "forwards":
            mode = "spinning"
            
            if   touch_activated: motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.8)        
            elif forwards_expire: pass
            elif gap_found:       motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 1.2)
            elif out_of_bounds:   motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.8)
            
            spinning_direction = 1 if randint(0, 100) < 99 else -1
            
            motors.run(0, 0, 0.1)
            motors.run(evac_state.SPEED_BASE * spinning_direction, -evac_state.SPEED_BASE * spinning_direction, randint(1, 3) / 10)

            spinning_start_time = time.perf_counter()
            spinning_time = randint(20, 40) /  10 # 10, 30, old
        
        elif time.perf_counter() - spinning_start_time > spinning_time and mode == "spinning":
            mode = "forwards"
            forwards_start_time = time.perf_counter()
            forwards_time = randint(30, 50) / 10
        
        # Handle movement
        if mode == "forwards":
            motors.run(evac_state.SPEED_FAST,  evac_state.SPEED_FAST)
            time_remaining = forwards_time - time.perf_counter() + forwards_start_time
            
        elif mode == "spinning":
            motors.run(evac_state.SPEED_BASE * spinning_direction, -evac_state.SPEED_BASE * spinning_direction)
            time_remaining = spinning_time - time.perf_counter() + spinning_start_time
        
        # Exit condition
        image = evac_camera.capture()
        display_image = image.copy()
        
        x, search_type = analyse(image, display_image, "default")
        if x is not None: return black_count, silver_count, x, search_type
        
        # Debug 
        if evac_state.DEBUG_MAIN: debug(["LOCATE", f"mode: {mode}", f"search: {search_type}", f"victims: {evac_state.victim_count}", f"claw: {claw.spaces}", f"Black {black_count} Silver: {silver_count}", f"Time: {time_remaining:.2f}"], [30, 20, 20, 20, 20, 20, 20])
        if evac_state.DISPLAY:    show(np.uint8(display_image), name="display", display=True)

    return black_count, silver_count, None, "default"

def route(black_count: int, silver_count: int, last_x: int, search_type: str, retry: bool = False) -> bool:
    MAX_RETRYS = 5
    
    start_time = time.perf_counter()
    max_route_time = 15
    retry_count = 0
    
    while listener.mode.value != 0:
        if evac_state.DEBUG_LOCATE: print("\tReading sensor stack")
        distance = laser_sensors.read([1])[0]
        if distance is None: continue
        
        silver_value = silver_sensor.read()
        touch_values = touch_sensors.read()
        gyro_value = gyroscope.read()
        
        if gyro_value is not None:
            _, pitch, _ = gyro_value
        
        black_count, silver_count = validate_gap(silver_value, black_count, silver_count)
        
        # Fail conditions: environemental
        touch_activated = sum(touch_values) != 2
        timeout            = time.perf_counter() - start_time > max_route_time
        gap_found          = black_count > evac_state.GAP_BLACK_COUNT or silver_count > evac_state.GAP_SILVER_COUNT
        out_of_bounds_up   = pitch >  10 if pitch is not None else False
        out_of_bounds_down = pitch < -10 if pitch is not None else False
        
        if gap_found: black_count = silver_count = 0
        if timeout or gap_found or out_of_bounds_up:
            motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.8)
            motors.run( evac_state.SPEED_BASE, -evac_state.SPEED_BASE, randint(3, 6) / 10)
            return False
        if out_of_bounds_down:
            motors.run(evac_state.SPEED_FAST, evac_state.SPEED_FAST, 0.8)
            motors.run( evac_state.SPEED_BASE, -evac_state.SPEED_BASE, randint(3, 6) / 10)
            return False
        
        if touch_activated and search_type in ["live", "dead"]:
            motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.8)
            motors.run( evac_state.SPEED_BASE, -evac_state.SPEED_BASE, randint(3, 6) / 10)
            return False
            
        # Capture image and perform analysis
        image = evac_camera.capture()
        display_image = image.copy()
        
        x, search_type = analyse(image, display_image, search_type, last_x)

        if search_type in ["live", "dead"]:
            evac_state.last_victim_time = time.perf_counter()

        if x is None:
            if not retry: return False
            else:
                retry_count += 1
                motors.run(0, 0, 0.1)
                for _ in range(0, 3): evac_camera.capture()

                if retry_count == MAX_RETRYS: return False
                else: continue
        
        movement.route(0.4, x, search_type)
        
        last_x = x
        print(f"time: {time.perf_counter() - evac_state.last_victim_time}, search type: {search_type}")
        
        # Exit conditions
        if search_type in ["red", "green"] and time.perf_counter() - evac_state.last_victim_time > evac_state.MIN_VICTIM_TRIANGLE_SWITCH_TIME:
            touch_values = touch_sensors.read()
            print(f"CHECKING TOUCH!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! {touch_values}")
            print(f"CHECKING TOUCH CONDITIONS: {sum(touch_values) != 2} {distance < evac_state.ROUTE_APPROACH_TRIANGLE_DISTANCE}")
            if sum(touch_values) != 2 or distance < evac_state.ROUTE_APPROACH_TRIANGLE_DISTANCE:
                touch_count = 0
    
                while listener.mode.value != 0:
                    touch_values = touch_sensors.read()
                    
                    if sum(touch_values) == 0:
                        touch_count += 1
                        
                    elif sum(touch_values) == 2:
                        touch_count = 0
                        motors.run(35, 35)
                        
                    elif touch_values[0] == 0:
                        motors.run(-12, 35)
                        touch_count = 0
                        
                    elif touch_values[1] == 0:
                        motors.run(35, -12)
                        touch_count = 0
                        
                    if touch_count > 5:
                        break

                motors.run(0, 0)
                
                return True
            
        elif distance < evac_state.ROUTE_APPROACH_VICTIM_DISTANCE and search_type in ["live", "dead"]:
            return True

        text = "ROUTING" if not retry else "ROUTING RETRY"
        if evac_state.DEBUG_ROUTE: debug( [text, f"{search_type}", f"x: {x} {last_x}", f"Counts: {black_count} {silver_count}"], [30, 20, 20, 20] )
        if evac_state.DISPLAY:     show(display_image, name="Display", display=True)
        
    return False
    
def grab(prev_insert: Optional[int]) -> tuple[bool, int]:    
    if "" not in claw.spaces: return False, None
    
    # Left: -1, Right: 1
    insert = -1 if claw.spaces[0] == "" else 1
    
    if claw.spaces[0] == claw.spaces[1] == "":
        if prev_insert == insert: insert = 1 if insert == -1 else -1
    
    debug( ["GRAB", "SETUP"], [30, 20] )
    # Setup
    motors.run(0, 0, 0.1)
    motors.run(         -evac_state.SPEED_GRAB,         -evac_state.SPEED_GRAB, 0.3)
    if insert == 1:  motors.run(-evac_state.SPEED_GRAB * insert, evac_state.SPEED_GRAB * insert, 0.2)
    if insert == -1: motors.run(-evac_state.SPEED_GRAB * insert, evac_state.SPEED_GRAB * insert, 0.35)
    motors.run(0, 0, 0.1)
    
    debug( ["GRAB", "CUP"], [30, 20] )
    # Cup
    claw.close(180 if insert == -1 else 0)
    claw.lift(0, 0.005)
    time.sleep(0.7)
    
    debug( ["GRAB", "LIFT"], [30, 20] )
    
    # Lift
    claw.close(90)
    claw.lift(180, 0.005)
    time.sleep(0.3)
    
    debug( ["GRAB", "VALIDATE"], [30, 20] )
    claw.read()
    debug( ["GRAB", f"{claw.spaces}"], [30, 20] )

    if claw.spaces[0 if insert == -1 else 1] != "": return True, None
    else:                                           return False, insert

def dump(search_type: str) -> None:
    TIME_STEP = 0.2
    
    motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.3)
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
        
        evac_state.victim_count += 1
        
    claw.close(90)
    claw.lift(180)
    time.sleep(1)
    
    claw.read()

def validate_gap(silver_value: int, black_count: int, silver_count: int) -> tuple[int, int]:
    silver_count += 1 if silver_value >= evac_state.SILVER_MIN else -1
    black_count  += 1 if silver_value <=  evac_state.BLACK_MAX else -1

    if silver_count < 0: silver_count = 0
    if black_count  < 0: black_count  = 0
    
    return black_count, silver_count

def align_line(align_type: str, align_count: int) -> None:
    if align_type == "silver":   

        motors.run( evac_state.SPEED_BASE * 0.7,  evac_state.SPEED_BASE * 0.7, 1)
        motors.run(-evac_state.SPEED_BASE * 0.3, -evac_state.SPEED_BASE * 0.3)
        left_silver, right_silver = False, False
        
        # Move backwards till 1 finds silver
        while (not left_silver and not right_silver) and listener.mode.value != 0:
            colour_values = colour_sensors.read()

            if colour_values[0] > evac_state.SILVER_MIN or colour_values[1] > evac_state.SILVER_MIN:
                left_silver = True
                print("Found left Silver")

            elif colour_values[3] > evac_state.SILVER_MIN or colour_values[4] > evac_state.SILVER_MIN:
                right_silver = True
                print("Found right silver")
        
        # Moving forwards slowly
        motors.run(evac_state.SPEED_BASE * 0.5, evac_state.SPEED_BASE * 0.5, 0.4)

        # Account for angled entry
        if(left_silver): 
            motors.run_until(7, -evac_state.SPEED_BASE * 0.5, colour_sensors.read, 4, ">=", evac_state.SILVER_MIN, "Right")
        
        # Repeat finding silver
        for i in range(align_count):
            forwards_speed = evac_state.SPEED_BASE * 0.5
            other_speed = 8
            
            motors.run      ( 0, 0, 0.2)
            motors.run_until(-forwards_speed,     other_speed, colour_sensors.read, 0, ">=", evac_state.SILVER_MIN, "LEFT SILVER")
            
            motors.run      ( 0, 0, 0.2)
            motors.run_until(    other_speed, -forwards_speed, colour_sensors.read, 4, ">=", evac_state.SILVER_MIN, "RIGHT SILVER")
            
            motors.run      (0, 0, 0.2)
            motors.run_until( forwards_speed,     other_speed, colour_sensors.read, 0, "<=", evac_state.SILVER_MIN, "LEFT WHITE")
            
            motors.run      ( 0, 0, 0.2)
            motors.run_until(    other_speed,  forwards_speed, colour_sensors.read, 4, "<=", evac_state.SILVER_MIN, "RIGHT WHITE")
                    
    elif align_type == "black":
        motors.run( evac_state.SPEED_BASE * 0.3,  evac_state.SPEED_BASE * 0.3)
        left_black, right_black = None, None
        
        while (left_black is None and right_black is None) and listener.mode.value != 0:
            colour_values = colour_sensors.read()

            if colour_values[0] < 40 or colour_values[1] < 40:
                left_black = True
                print("Found left black")
                
            elif colour_values[3] < 40 or colour_values[4] < 40:
                right_black = True
                print("Found right black")

        if (left_black): 
            motors.run_until(-5, evac_state.SPEED_BASE * 0.5, colour_sensors.read, 4, "<=", 30, "Right")
            
        for i in range(align_count):
            motors.run(0, 0, 0.2)
            motors.run_until(-evac_state.SPEED_BASE * 0.5, -5, colour_sensors.read, 0, ">=", 60, "Left")
            motors.run(0, 0, 0.2)
            motors.run_until(-5, -evac_state.SPEED_BASE * 0.5, colour_sensors.read, 4, ">=", 60, "Right")
            motors.run(0, 0, 0.2)
            motors.run_until(evac_state.SPEED_BASE * 0.5, -5, colour_sensors.read, 0, "<=", 30, "Left")
            motors.run(0, 0, 0.2)
            motors.run_until(-5, evac_state.SPEED_BASE * 0.5, colour_sensors.read, 4, "<=", 30, "Right")

        for i in range(0, int(evac_state.SPEED_BASE * 0.7)):
            motors.run(-evac_state.SPEED_BASE*0.7+i, -evac_state.SPEED_BASE*0.7+i, 0.03)

def leave():
    silver_count = black_count = 0
    led.on()
    
    touch_count = 0
    
    while listener.mode.value != 0:
        touch_values = touch_sensors.read()
        
        if sum(touch_values) == 0:
            touch_count += 1
            
        elif sum(touch_values) == 2:
            touch_count = 0
            motors.run(35, 35)
            
        elif touch_values[0] == 0:
            motors.run(-12, 35)
            touch_count = 0
            
        elif touch_values[1] == 0:
            motors.run(35, -12)
            touch_count = 0
            
        if touch_count > 5:
            break

    motors.run(0, 0)

    motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST, touch_sensors.read, 0, "==", 0, "FORWARDS LEFT")
    motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST, touch_sensors.read, 1, "==", 0, "FORWARDS RIGHT")
    motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.3)
    motors.run( evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 1)
    
    while listener.mode.value != 0:
        movement.wall_follow()
        silver_value = silver_sensor.read()
        # black_count, silver_count = validate_gap(silver_value, black_count, silver_count)
        colour_value = colour_sensors.read()[2]
        black_count, silver_count = validate_gap(colour_value, black_count, silver_count)

        if black_count >= 5:
            debug(["EXITING", "FOUND EXIT!"], [24, 30])
            motors.run(evac_state.SPEED_BASE, evac_state.SPEED_BASE, 0.5)
        
            # motors.run(20, 20)
            # left_black = right_black = False
            # while True:
            #     colour_values = colour_sensors.read()
            #     if any(colour_values[0, 1]) < 40:
            #         motors.run(0, 20)
            #         left_black = True
            #     elif any(colour_values[3, 4]) < 40:
            #         motors.run(20, 0)
            #         right_black = True
                
            #     if left_black and right_black:
            #         break
                    
                    
            # align_line("black", 2)
            
            break
        
        
        elif silver_count >= 15:
            debug(["EXITING", "FOUND SILVER"], [24, 30])
            
            motors.run(0, 0, 0.3)
            motors.run(-evac_state.SPEED_BASE, -evac_state.SPEED_BASE, 1.7)
            motors.run(0, 0, 0.3)
            motors.run(evac_state.SPEED_BASE, -evac_state.SPEED_BASE, 1.7)
            motors.run(0, 0, 0.3)

            while listener.mode.value != 0:
                print("running till wall")
                touch_count = 0
    
                while listener.mode.value != 0:
                    touch_values = touch_sensors.read()
                    
                    if sum(touch_values) == 0:
                        touch_count += 1
                        
                    elif sum(touch_values) == 2:
                        touch_count = 0
                        motors.run(35, 35)
                        
                    elif touch_values[0] == 0:
                        motors.run(-12, 35)
                        touch_count = 0
                        
                    elif touch_values[1] == 0:
                        motors.run(35, -12)
                        touch_count = 0
                        
                    if touch_count > 5:
                        break

                motors.run(0, 0)
                motors.run( evac_state.SPEED_BASE,  evac_state.SPEED_BASE, 1)
                motors.run(-evac_state.SPEED_BASE, -evac_state.SPEED_BASE, 0.5)
                motors.run( evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 1)
                
                motors.run_until(evac_state.SPEED_BASE,  -evac_state.SPEED_BASE, laser_sensors.read, 0, "<=", 10)

                break

##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################

evac_state = EvacuationState()
op_search = Search(evac_state, evac_camera)
movement = Movement(evac_state, evac_camera, touch_sensors, colour_sensors, motors)

def main() -> None:
    black_count = silver_count = 0    
    prev_insert = None

    led.off()
    
    oled_display.clear()
    oled_display.text("Evac start", 0, 0)
    for _ in range(0, 2): evac_camera.capture()

    motors.run(evac_state.SPEED_FAST, evac_state.SPEED_FAST, 1.5)
    motors.run(0, 0, 1) 

    while evac_state.victim_count != 3 and listener.mode.value != 0:
        oled_display.clear()
        oled_display.text(f"locate: {evac_state.victim_count}", 0, 0)
        
        black_count, silver_count, x, search_type = locate(black_count, silver_count)
        if x is None:
            continue
        
        oled_display.text(f"route {search_type}", 0, 10)
        route_success = route(black_count, silver_count, x, search_type)
        if not route_success: continue
        
        if search_type in ["red", "green"]:
            # motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST* 0.8, touch_sensors.read, 0, "==", 0, "TRIANGLE TOUCH")
            # motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST* 0.8, touch_sensors.read, 1, "==", 0, "TRIANGLE TOUCH")
            
            motors.run(0, 0, 0.1)
            
            # motors.run(-evac_state.SPEED_FAST     , -evac_state.SPEED_FAST * 0.5, 0.8)
            # motors.run( evac_state.SPEED_FAST * 0.5, evac_state.SPEED_FAST      , 0.8)
            
            # # black_count, silver_count, x, search_type = locate(black_count, silver_count)
            # for _ in range(0, 5): evac_camera.capture()
            # motors.run(0, 0, 0.1)

            # route_success = route(black_count, silver_count, None, search_type, retry=True)
            # if not route_success: continue
            
            # motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST, touch_sensors.read, 0, "==", 0, "TRIANGLE TOUCH")
            # motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST, touch_sensors.read, 1, "==", 0, "TRIANGLE TOUCH")
    
            oled_display.text(f"dump", 0, 20)
            dump(search_type)
            
            motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.2)
            motors.run( evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 1)
            
        else:
            oled_display.text(f"grab", 0, 20)
            grab_success, prev_insert = grab(prev_insert)

            if not grab_success:
                motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 1.2)
                motors.run(0, 0, 0.5)
                continue
    
    if listener.mode.value != 0:
        oled_display.text(f"leaving", 0, 20, clear=True)
        print("LEAVING")
        
        motors.run(0, 0, 0.5)
        leave()
        
        motors.run(0, 0)
        print("EXITED!")
            
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################

if __name__ == "__main__":
    listener.mode.value = 2
    # main()
    
    leave()
    motors.run(0, 0)