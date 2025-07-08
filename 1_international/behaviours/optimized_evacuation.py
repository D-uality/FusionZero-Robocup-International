if __name__ == "__main__":
    import sys, pathlib
    root = pathlib.Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(root))

from core.shared_imports import time, np, cv2, Optional, randint, socket, getpass
from hardware.robot import *
from core.utilities import *

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
        username = getpass.getuser()
        hostname = socket.gethostname()
        self.user_at_host = f"{username}@{hostname}"
        
        if self.user_at_host == "frederick@raspberrypi":
            self.SPEED_BASE  = 35
            self.SPEED_FAST  = 45
            self.SPEED_ROUTE = 25
            self.SPEED_GRAB  = 30
        else:
            self.SPEED_BASE  = 35
            self.SPEED_FAST  = 45
            self.SPEED_ROUTE = 20
            self.SPEED_GRAB  = 30
            
        # Locate settings
        self.DEBUG_LOCATE = False
        
        # Analyse settings
        self.DEBUG_ANALYSE = False
        
        # Route settings
        self.DEBUG_ROUTE = True
        self.ROUTE_APPROACH_DISTANCE = 7
        self.last_victim_time = 0
        self.MIN_VICTIM_TRIANGLE_SWITCH_TIME = 1
        
        # Gap handling
        self.SILVER_MIN       = 120
        self.BLACK_MAX        = 30
        self.GAP_BLACK_COUNT  = 3
        self.GAP_SILVER_COUNT = 3

class Search():
    def __init__(self, evac_state: EvacuationState, evac_camera: EvacuationCamera):
        # Debug
        self.DISPLAY: bool = evac_state.DISPLAY
        
        self.DEBUG_LIVE: bool       = False
        self.DEBUG_DEAD: bool       = False
        self.DEBUG_TRIANGLES: bool  = True
        
        self.TIMING_LIVE: bool      = False
        self.TIMING_DEAD: bool      = False
        self.TIMING_TRIANGLES: bool = False
        
        # General settings
        self.CROP_SIZE = 200
        self.IMG_CENTRE_X = evac_camera.width / 2
        
        # Live settings
        self.LIVE_THRESHOLD = 245
        self.LIVE_DILATE_KERNAL = np.ones((15, 15), np.uint8)
        
        # Dead settings
        self.DEAD_THRESHOLD_BLACK = 25
        self.DEAD_THRESHOLD_SILVER = 160
        self.DEAD_DILATE_WHITE_KERNAL = np.ones((3, 3), np.uint8)
        self.DEAD_DILATE_GREEN_KERNAL = np.ones((3, 3), np.uint8)
        self.DEAD_DILATE_MAXIMA_KERNAL = np.ones((7, 7), np.uint8)
        self.DEAD_MORPH_KERNAL = (7, 7)
        self.DEAD_COLOUR_VARIANCE_THRESHOLD = 25
        self.DEAD_MIN_AREA = 200
        self.DEAD_MAX_AREA = 50000
        self.DEAD_MIN_RADIUS = 5
        self.DEAD_MAX_RADIUS = 110
        self.DEAD_TRANSFORM_SPLITTING_THRESHOLD = 8000
        self.CIRCULARITY_THRESHOLD = 0.6
        self.FILL_RATIO_THRESHOLD = 0.5
        
        # Triangle settings
        self.TRIANGLE_DILATE_KERNAL = np.ones((5, 5), np.uint8)
        self.TRIANGLE_MIN_AREA = 800
        
    def live(self, image: np.ndarray, display: np.ndarray, last_x: Optional[int]) -> Optional[int]:
        if image is None or image.size == 0: return None
        if self.TIMING_LIVE: start_time = time.perf_counter()
            
        # Crop according to last x
        if last_x is not None:
            x_lower = max(0, last_x - self.CROP_SIZE)
            x_upper = min(image.shape[1], last_x + self.CROP_SIZE)
            working_image = image[:, x_lower:x_upper]
            
        else:
            working_image = image.copy()
            x_lower = 0
            
        if self.TIMING_LIVE: crop_time = time.perf_counter()
        
        if self.TIMING_LIVE: preprocess_time = time.perf_counter()
        
        # Filter for spectral highlights
        working_image = cv2.dilate(working_image, self.LIVE_DILATE_KERNAL, iterations=1)
        spectral_highlights = cv2.inRange(working_image, (self.LIVE_THRESHOLD + 5, self.LIVE_THRESHOLD, self.LIVE_THRESHOLD), (255, 255, 255))
        if self.TIMING_LIVE: threshold_time = time.perf_counter()
        
        # Connected components analysis
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(spectral_highlights, connectivity=8)
        
        if num_labels <= 1:  # Only background found
            if self.TIMING_LIVE:
                total_time = time.perf_counter()
                print(f"live() | Total: {(total_time - start_time)*1000:.2f}ms | Crop: {(crop_time - start_time)*1000:.2f}ms | Preprocess: {(preprocess_time - crop_time)*1000:.2f}ms | Threshold: {(threshold_time - preprocess_time)*1000:.2f}ms | Status: no_components")
            
            return None
        
        if self.TIMING_LIVE: components_time = time.perf_counter()
        
        # Extract component data (skip background at index 0)
        areas = stats[1:, cv2.CC_STAT_AREA]
        centroid_y = centroids[1:, 1]
        centroid_x = centroids[1:, 0]
        
        # Filter valid components
        valid_mask = (
            (areas > 200) & 
            (areas < 3000) & 
            (centroid_y > 5) & 
            (centroid_y < 80)
        )
        
        if not np.any(valid_mask):
            if self.TIMING_LIVE:
                total_time = time.perf_counter()
                print(f"live() | Total: {(total_time - start_time)*1000:.2f}ms | Crop: {(crop_time - start_time)*1000:.2f}ms | Preprocess: {(preprocess_time - crop_time)*1000:.2f}ms | Threshold: {(threshold_time - preprocess_time)*1000:.2f}ms | Components: {(components_time - threshold_time)*1000:.2f}ms | Validation: {(total_time - components_time)*1000:.2f}ms | Status: no_valid")
            
            return None
        
        if self.TIMING_LIVE: validation_time = time.perf_counter()
        
        # Select best component
        valid_areas = areas[valid_mask]
        valid_centroid_x = centroid_x[valid_mask]
        
        if last_x is None:
            # Select largest area
            selected_idx = np.argmax(valid_areas)
            
        else:  
            # Select closest to image center (in global coordinates)
            global_centroid_x = valid_centroid_x + x_lower
            distances = np.abs(global_centroid_x - self.IMG_CENTRE_X)
            selected_idx = np.argmin(distances)
        
        if self.TIMING_LIVE: selection_time = time.perf_counter()
        
        # Get final center position
        final_centre_x = int(valid_centroid_x[selected_idx] + x_lower)
        
        # Display if enabled
        if self.DISPLAY:
            # Create mask for selected component
            selected_label = np.where(valid_mask)[0][selected_idx] + 1  # +1 because we skipped background
            component_mask = (labels == selected_label).astype(np.uint8) * 255
            
            if x_lower > 0:
                # Create full-size mask for display
                full_mask = np.zeros((display.shape[0], display.shape[1]), dtype=np.uint8)
                full_mask[:, x_lower:x_lower + component_mask.shape[1]] = component_mask
                contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            else:
                contours, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                cv2.drawContours(display, contours, -1, (0, 255, 0), 2)
                
            if self.DEBUG_LIVE:
                component_mask_colored = cv2.cvtColor(component_mask, cv2.COLOR_GRAY2BGR)
                cv2.drawContours(component_mask_colored, [contours[0]] if contours else [], -1, (0, 255, 0), 2)
                show(component_mask_colored, "live_working_image")
        
        if self.TIMING_LIVE: 
            drawing_time = time.perf_counter()
            end_time = time.perf_counter()
            print(f"live() | Total: {(end_time - start_time)*1000:.2f}ms | Crop: {(crop_time - start_time)*1000:.2f}ms | Preprocess: {(preprocess_time - crop_time)*1000:.2f}ms | Threshold: {(threshold_time - preprocess_time)*1000:.2f}ms | Components: {(components_time - threshold_time)*1000:.2f}ms | Validation: {(validation_time - components_time)*1000:.2f}ms | Selection: {(selection_time - validation_time)*1000:.2f}ms | Drawing: {(drawing_time - selection_time)*1000:.2f}ms | Final: {(end_time - drawing_time)*1000:.2f}ms | Status: success")
        
        if self.DEBUG_LIVE:
            print(f"Selected component - Area: {valid_areas[selected_idx]}, Center: {final_centre_x}")
        
        return final_centre_x
    
    def dead(self, image: np.ndarray, display: np.ndarray, last_x: Optional[int] = None) -> Optional[int]:
        if image is None or image.size == 0: return None
            
        if self.TIMING_DEAD:
            start_time = time.perf_counter()
        
        # Crop according to last x
        if last_x is not None:
            x_lower = max(0, last_x - self.CROP_SIZE)
            x_upper = min(image.shape[1], last_x + self.CROP_SIZE)
            working_image = image[:, x_lower:x_upper] if x_upper > x_lower else image
            
        else:
            working_image = image
            x_lower = 0
        
        # Combined preprocessing - do all color operations in fewer passes
        hsv_image = cv2.cvtColor(working_image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_image, (50, 120, 50), (70, 255, 255))
        green_mask = cv2.dilate(green_mask, self.DEAD_DILATE_GREEN_KERNAL, iterations=1)
        
        # Apply green and white normalization in combined pass
        working_image = working_image.copy()
        working_image[green_mask > 0] = [41, 41, 41]
        
        # Convert to grayscale once for both white and dark mask detection
        grey_image = cv2.cvtColor(working_image, cv2.COLOR_BGR2GRAY)
        
        # Apply green normalization to grayscale as well
        grey_image[green_mask > 0] = 41
        
        # White mask detection and normalization
        white_mask = grey_image > self.DEAD_THRESHOLD_SILVER
        white_mask = cv2.dilate(white_mask.astype(np.uint8) * 255, self.DEAD_DILATE_WHITE_KERNAL, iterations=1)
        
        # Apply normalization to both color and grayscale images
        working_image[white_mask > 0] = [160, 160, 160]
        grey_image[white_mask > 0] = 160
        
        if self.TIMING_DEAD: preprocess_time = time.perf_counter()
        
        # Step 1: Create initial dark mask but with color rejection
        # Only keep pixels that are dark AND neutral (not colored shadows)
        dark_mask = grey_image < self.DEAD_THRESHOLD_BLACK
        
        # Color rejection - remove non-neutral dark areas
        b, g, r = cv2.split(working_image)
        color_variance = np.maximum(np.maximum(np.abs(b.astype(int) - g.astype(int)), np.abs(g.astype(int) - r.astype(int))), np.abs(r.astype(int) - b.astype(int)))
        neutral_mask = color_variance < self.DEAD_COLOUR_VARIANCE_THRESHOLD  # Threshold for neutral colors
        
        # Combine dark and neutral masks
        candidate_mask = dark_mask & neutral_mask
        candidate_mask = candidate_mask.astype(np.uint8) * 255
        
        if self.DEBUG_DEAD: show(candidate_mask, name="candidate_mask", display=True)
        
        # Step 2: Morphological opening to remove thin shadows/connections
        opening_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, self.DEAD_MORPH_KERNAL)
        opened_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_OPEN, opening_kernel)
        
        if self.DEBUG_DEAD: show(opened_mask, "opened_mask")
        
        # Step 3: Connected components analysis
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(opened_mask, connectivity=8)
        
        if num_labels <= 1:  # Only background
            if self.TIMING_DEAD:
                print(f"dead() | Preprocess: {(preprocess_time-start_time)*1000:.1f}ms | Total: {(preprocess_time-start_time)*1000:.1f}ms | Status: no_components")
            return None
        
        if self.TIMING_DEAD:
            detection_time = time.perf_counter()
        
        # Step 4: Process each component
        valid_candidates = []
        
        for i in range(1, num_labels):  # Skip background (label 0)
            area = stats[i, cv2.CC_STAT_AREA]
            if self.DEBUG_DEAD: print(f"\tArea: {area}")
            
            # Reject based on area
            if area < self.DEAD_MIN_AREA or area > self.DEAD_MAX_AREA: continue
                
            # Get component mask
            component_mask = (labels == i).astype(np.uint8) * 255
            
            # For large components, try distance transform splitting
            if area > self.DEAD_TRANSFORM_SPLITTING_THRESHOLD:
                # Distance transform to find peaks
                dist_transform = cv2.distanceTransform(component_mask, cv2.DIST_L2, 5)
                
                # Find local maxima (potential circle centers)
                local_maxima = cv2.dilate(dist_transform, self.DEAD_DILATE_MAXIMA_KERNAL) == dist_transform
                local_maxima = local_maxima & (dist_transform > self.DEAD_MIN_RADIUS)
                
                # Get peak locations
                peak_locations = np.where(local_maxima)
                
                if len(peak_locations[0]) > 0:
                    # Process each peak as a potential circle
                    for py, px in zip(peak_locations[0], peak_locations[1]):
                        radius = int(dist_transform[py, px])
                        if self.DEAD_MIN_RADIUS <= radius <= self.DEAD_MAX_RADIUS:
                            valid_candidates.append((px, py, radius, area))
                else:
                    # Fallback: use centroid
                    cx, cy = int(centroids[i, 0]), int(centroids[i, 1])
                    est_radius = int(np.sqrt(area / np.pi))
                    if self.DEAD_MIN_RADIUS <= est_radius <= self.DEAD_MAX_RADIUS:
                        valid_candidates.append((cx, cy, est_radius, area))
            else:
                # Small components: use centroid and estimate radius
                cx, cy = int(centroids[i, 0]), int(centroids[i, 1])
                est_radius = int(np.sqrt(area / np.pi))
                if self.DEAD_MIN_RADIUS <= est_radius <= self.DEAD_MAX_RADIUS:
                    valid_candidates.append((cx, cy, est_radius, area))
        
        if self.DEBUG_DEAD: print(f"Valid candidates: {valid_candidates}")
        
        if not valid_candidates:
            if self.TIMING_DEAD:
                print(f"dead() | Preprocess: {(preprocess_time-start_time)*1000:.1f}ms | Detection: {(detection_time-preprocess_time)*1000:.1f}ms | Total: {(detection_time-start_time)*1000:.1f}ms | Status: no_valid")
            return None
        
        if self.TIMING_DEAD: validation_time = time.perf_counter()
        
        # Step 5: Circularity and fill-ratio validation
        final_candidates = []
        
        for cx, cy, radius, area in valid_candidates:
            # Create circular mask for this candidate
            circle_mask = np.zeros_like(opened_mask)
            cv2.circle(circle_mask, (cx, cy), radius, 255, -1)
            
            # Calculate intersection with detected dark regions
            intersection = cv2.bitwise_and(opened_mask, circle_mask)
            intersection_area = np.sum(intersection > 0)
            
            # Calculate fill ratio
            expected_area = np.pi * radius * radius
            fill_ratio = intersection_area / expected_area
            
            # Circularity test using contour
            intersection_contours, _ = cv2.findContours(intersection, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if intersection_contours:
                largest_contour = max(intersection_contours, key=cv2.contourArea)
                perimeter = cv2.arcLength(largest_contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * cv2.contourArea(largest_contour) / (perimeter * perimeter)
                else:
                    circularity = 0
            else:
                circularity = 0
            
            # Accept if circular enough and well-filled and low enough y-coordinate
            if self.DEBUG_DEAD: print(f"\tCircularity: {circularity:.2f} | Fill Ratio: {fill_ratio:.2f}")
            if circularity > self.CIRCULARITY_THRESHOLD and fill_ratio > self.FILL_RATIO_THRESHOLD and cy > 30:
                final_candidates.append((cx, cy, radius, fill_ratio * circularity))
        
        if not final_candidates:
            if self.TIMING_DEAD:
                print(f"dead() | Preprocess: {(preprocess_time-start_time)*1000:.1f}ms | Detection: {(detection_time-preprocess_time)*1000:.1f}ms | Validation: {(validation_time-detection_time)*1000:.1f}ms | Total: {(validation_time-start_time)*1000:.1f}ms | Status: no_final")
            return None
        
        if self.TIMING_DEAD:
            selection_time = time.perf_counter()
        
        # Select best candidate
        if last_x is not None:
            crop_center = working_image.shape[1] / 2
            best_candidate = min(final_candidates, key=lambda c: abs(c[0] - crop_center))
        else:
            best_candidate = max(final_candidates, key=lambda c: c[3])  # Best score
        
        x, y, r = best_candidate[:3]
        centre_x = x + x_lower
        
        # Display results
        if self.DISPLAY:
            cv2.circle(display, (centre_x, y), r, (0, 255, 0), 2)
            cv2.circle(display, (centre_x, y), 1, (0, 255, 0), 2)
        
        if self.TIMING_DEAD:
            print(f"dead() | Preprocess: {(preprocess_time-start_time)*1000:.1f}ms | Detection: {(detection_time-preprocess_time)*1000:.1f}ms | Validation: {(validation_time-detection_time)*1000:.1f}ms | Selection: {(selection_time-validation_time)*1000:.1f}ms | Total: {(selection_time-start_time)*1000:.1f}ms | Status: success")
        
        return centre_x

    def triangle(self, image: np.ndarray, display_image: np.ndarray, triangle: str) -> tuple[Optional[int], Optional[int]]:
        if image is None or image.size == 0: return None, None
        if self.TIMING_TRIANGLES: start_time = time.perf_counter()

        # Crop the image to remove the top 30 pixels
        image = image[30:, :]
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.TIMING_TRIANGLES: hsv_time = time.perf_counter()
        
        # Select which mask to use
        if triangle == "red":
            mask_lower = cv2.inRange(hsv_image, (  0, 120, 0), ( 20, 255, 255))
            mask_upper = cv2.inRange(hsv_image, (160, 120, 0), (179, 255, 255))
            mask = cv2.bitwise_or(mask_lower, mask_upper)
        else:
            mask = cv2.inRange(hsv_image, (50, 120, 15), (85, 255, 255))
        
        if self.TIMING_TRIANGLES: mask_time = time.perf_counter()
        
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
            for contour in contours: print(cv2.contourArea(contour))
        
        largest_contour = max(contours, key=cv2.contourArea)
        contour_area = cv2.contourArea(largest_contour)
        
        if self.DEBUG_TRIANGLES: print(contour_area)
        
        if contour_area < self.TRIANGLE_MIN_AREA:
            if self.TIMING_TRIANGLES:
                total_time = time.perf_counter()
                print(f"triangle() | HSV: {(hsv_time-start_time)*1000:.1f}ms | Mask: {(mask_time-hsv_time)*1000:.1f}ms | Dilate: {(dilate_time-mask_time)*1000:.1f}ms | Contours: {(contours_time-dilate_time)*1000:.1f}ms | Total: {(total_time-start_time)*1000:.1f}ms | Status: too_small")
            return None, None
        
        if self.TIMING_TRIANGLES: area_time = time.perf_counter()
        
        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        if h > w or y + h/2 > evac_camera.height / 2:
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
        GAP_DISTANCE = 13                               # Threshold to classify for gap
        OFFSET = 3                                      # Wall following distance
        MAX_TURN = evac_state.SPEED_BASE * 0.5          # Max turning speed
        
        touch_values = touch_sensors.read()
        distance = laser_sensors.read([0])[0]
        
        # Laser sensors failed
        if distance is None: return None
        
        error = distance - OFFSET
        raw_turn = KP * error
        
        # Touch
        if sum(touch_values) != 2:
            turn_time = 0.5 if distance < GAP_DISTANCE else 0.65

            motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.25)
            motors.run( evac_state.SPEED_FAST, -evac_state.SPEED_FAST, turn_time)

        # Turn into gaps if leaving, uncap max_turn and increase the raw_turn
        if distance > GAP_DISTANCE:
            raw_turn = raw_turn * 12
            MAX_TURN = evac_state.SPEED_BASE * 4
        
        # Project an artificial pivot point
        effective_turn = raw_turn * (1 + 2 / max(distance, OFFSET))
        effective_turn = min(max(effective_turn, -MAX_TURN), MAX_TURN)

        v1 = evac_state.SPEED_BASE - effective_turn
        v2 = evac_state.SPEED_BASE + effective_turn

        motors.run(v1, v2)

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
        live_x = search.live(image, display_image, last_x)
        if live_x is not None: return live_x, "live"

    if search_type in ["dead", "green", "default"] and dead_enable:
        dead_x = search.dead(image, display_image, last_x)
        if dead_x is not None: return dead_x, "dead"
    
    if search_type in ["green", "default"] and green_enable:
        x, _ = search.triangle(image, display_image, "green")
        if x is not None: return x, "green"
    
    if search_type in ["red", "default"] and red_enable:
        x, _  = search.triangle(image, display_image, "red")
        if x is not None: return x, "red"
    
    return None, search_type

def locate(black_count: int, silver_count: int) -> tuple[int, int, int, str]:
    mode = "forwards" 
    spinning_start_time = time.perf_counter()
    forwards_start_time = time.perf_counter()
    spinning_time = randint(30, 80) / 10
    forwards_time = 5
    spinning_direction = 1
    
    while True:
        if evac_state.DEBUG_LOCATE: print("\tReading sensor stack")
        colour_values = colour_sensors.read()
        silver_value = silver_sensor.read()
        touch_values = touch_sensors.read()
        _, pitch, _ = gyroscope.read()
        
        black_count, silver_count = validate_gap(silver_value, black_count, silver_count)
        
        touch_activated = sum(touch_values) != 2
        forwards_expire = time.perf_counter() - forwards_start_time > forwards_time
        gap_found       = black_count > evac_state.GAP_BLACK_COUNT or silver_count > evac_state.GAP_SILVER_COUNT
        out_of_bounds   = pitch > 10 if pitch is not None else False
        
        if evac_state.DEBUG_LOCATE: print(f"\tForwards -> spinning conditions: {touch_activated} {forwards_expire} {gap_found}")
    
        if (touch_activated or forwards_expire or gap_found or out_of_bounds) and mode == "forwards":
            mode = "spinning"
            
            if   touch_activated: motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.8)        
            elif forwards_expire: pass
            elif gap_found:       motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 1.2)
            elif out_of_bounds:   motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.8)
            
            spinning_direction = 1 if randint(0, 100) < 90 else -1
            
            motors.run(0, 0, 0.1)
            motors.run(evac_state.SPEED_BASE * spinning_direction, -evac_state.SPEED_BASE * spinning_direction, randint(1, 5) / 10)

            backwards = True if randint(0, 100) < 35 else False
            spinning_start_time = time.perf_counter()
            
            # if backwards:
            #     spinning_time = randint(30, 60) / 10
            # else:
            spinning_time = randint(5, 30) / 10
        
        elif time.perf_counter() - spinning_start_time > spinning_time and mode == "spinning":
            mode = "forwards"
            forwards_start_time = time.perf_counter()
        
        if   mode == "forwards": motors.run(evac_state.SPEED_FAST,  evac_state.SPEED_FAST)
        elif mode == "spinning": motors.run(evac_state.SPEED_BASE * spinning_direction, -evac_state.SPEED_BASE * spinning_direction)
        
        if evac_state.DEBUG_LOCATE:
            if mode == "forwards": print(f"\tTime remaining: {forwards_time - time.perf_counter() + forwards_start_time:.2f}")
            if mode == "spinning": print(f"\tTime remaining: {spinning_time - time.perf_counter() + spinning_start_time:.2f}")
        
        if mode == "forwards": time_remaining = forwards_time - time.perf_counter() + forwards_start_time
        if mode == "spinning": time_remaining = spinning_time - time.perf_counter() + spinning_start_time
        
        image = evac_camera.capture()
        display_image = image.copy()
        
        x, search_type = analyse(image, display_image, "default")
        if x is not None: return black_count, silver_count, x, search_type
        
        if evac_state.DEBUG_MAIN: debug(["LOCATE", f"mode: {mode}", f"search: {search_type}", f"victims: {evac_state.victim_count}", f"claw: {claw.spaces}", f"Black {black_count} Silver: {silver_count}", f"Time: {time_remaining:.2f}"], [30, 20, 20, 20, 20, 20, 20])
        if evac_state.DISPLAY:    show(np.uint8(display_image), name="display", display=True)

def route(black_count: int, silver_count: int, last_x: int, search_type: str, retry: bool = False) -> bool:
    MAX_RETRYS = 5
    
    start_time = time.perf_counter()
    max_route_time = 15
    retry_count = 0
    
    while True:
        if evac_state.DEBUG_LOCATE: print("\tReading sensor stack")
        distance = laser_sensors.read([1])[0]
        if distance is None: continue
        
        colour_values = colour_sensors.read()
        silver_value = silver_sensor.read()
        touch_values = touch_sensors.read()
        _, pitch, _ = gyroscope.read()
        
        black_count, silver_count = validate_gap(silver_value, black_count, silver_count)
        
        touch_activated = sum(touch_values) != 2
        timeout         = time.perf_counter() - start_time > max_route_time
        gap_found       = black_count > evac_state.GAP_BLACK_COUNT or silver_count > evac_state.GAP_SILVER_COUNT
        out_of_bounds   = pitch > 10 if pitch is not None else False
        
        if touch_activated or timeout or gap_found or out_of_bounds:
            motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.8)
            motors.run(evac_state.SPEED_BASE, -evac_state.SPEED_BASE, randint(10, 30) / 10)
            return False
        
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
        
        movement.route(0.3, x, search_type)
        
        last_x = x
        print(f"time: {time.perf_counter() - evac_state.last_victim_time}, search type: {search_type}")
        if distance < evac_state.ROUTE_APPROACH_DISTANCE:
            if search_type in ["red", "green"] and time.perf_counter() - evac_state.last_victim_time > evac_state.MIN_VICTIM_TRIANGLE_SWITCH_TIME: 
                return True
            elif search_type in ["live", "dead"]:
                return True
        

        text = "ROUTING" if not retry else "ROUTING RETRY"
        if evac_state.DEBUG_ROUTE: debug( [text, f"{search_type}", f"x: {x} {last_x}", f"Counts: {black_count} {silver_count}"], [30, 20, 20, 20] )
        if evac_state.DISPLAY:     show(display_image, name="Display", display=True)
        
def grab(prev_insert: Optional[int]) -> tuple[bool, int]:    
    if "" not in claw.spaces: return False, None
    
    # Left: -1, Right: 1
    insert = -1 if claw.spaces[0] == "" else 1
    if prev_insert is None and prev_insert == insert: insert = 1 if insert == -1 else -1
    
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
    time.sleep(0.3)
    
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
    silver_count += 1 if silver_value >= evac_state.SILVER_MIN else (silver_count - 1) 
    black_count  += 1 if silver_value <=  evac_state.BLACK_MAX else (black_count - 1)

    if silver_count < 0: silver_count = 0
    if black_count  < 0: black_count  = 0
    
    return black_count, silver_count

def align_line(align_type: str, align_count: int) -> None:
    if align_type == "silver":   

        motors.run( evac_state.SPEED_BASE * 0.7,  evac_state.SPEED_BASE * 0.7, 1)
        motors.run(-evac_state.SPEED_BASE * 0.3, -evac_state.SPEED_BASE * 0.3)
        left_silver, right_silver = False, False
        
        # Move backwards till 1 finds silver
        while not left_silver and not right_silver:
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
        motors.run(-evac_state.SPEED_BASE * 0.7, -evac_state.SPEED_BASE * 0.7, 1)
        motors.run( evac_state.SPEED_BASE * 0.3,  evac_state.SPEED_BASE * 0.3)
        left_black, right_black = None, None
        
        while left_black is None and right_black is None:
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
    
    motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST, touch_sensors.read, 0, "==", 0, "FORWARDS LEFT")
    motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST, touch_sensors.read, 1, "==", 0, "FORWARDS RIGHT")
    motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.3)
    motors.run( evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 1)
    
    while True:
        movement.wall_follow()
        colour_values = colour_sensors.read()
        silver_value = silver_sensor.read()
        black_count, silver_count = validate_gap(silver_value, black_count, silver_count)

        if black_count >= 5:
            debug(["EXITING", "FOUND EXIT!"], [24, 30])
            align_line("black", 2)
            break
        
        elif silver_count >= 5:
            debug(["EXITING", "FOUND SILVER"], [24, 30])
            
            motors.run(0, 0, 0.3)
            align_line("silver", 2)
            motors.run(-evac_state.SPEED_BASE, -evac_state.SPEED_BASE, 1.7)
            motors.run(0, 0, 0.3)
            motors.run(evac_state.SPEED_BASE, -evac_state.SPEED_BASE, 2.2)
            motors.run(0, 0, 0.3)

            while True:
                distance = laser_sensors.read([0])[0]
                touch_values = touch_sensors.read()
                
                debug(["MOVING TILL WALL", f"{distance}", f"{touch_values}"], [24, 10, 10])
                motors.run(22, 22)

                if distance <= 20:
                    break
                
                if sum(touch_values) != 2:
                    motors.run( evac_state.SPEED_BASE,  evac_state.SPEED_BASE, 1)
                    motors.run(-evac_state.SPEED_BASE, -evac_state.SPEED_BASE, 0.5)
                    motors.run( evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 1)
                    
                    motors.run_until(evac_state.SPEED_BASE,  -evac_state.SPEED_BASE, laser_sensors.read, 0, "<=", 10)

                    break

##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################

evac_state = EvacuationState()
search = Search(evac_state, evac_camera)
movement = Movement(evac_state, evac_camera, touch_sensors, colour_sensors, motors)

def main() -> None:
    black_count = silver_count = 0
    evac_state.victim_count = 0
    prev_insert = None

    led.off()
    
    for _ in range(0, 2): evac_camera.capture()
    motors.run(evac_state.SPEED_FAST, evac_state.SPEED_FAST, 1.5)
    motors.run(0, 0, 1) 

    while evac_state.victim_count != 3:
        black_count, silver_count, x, search_type = locate(black_count, silver_count)
        
        route_success = route(black_count, silver_count, x, search_type)
        if not route_success: continue
        
        if search_type in ["red", "green"]:
            motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST, touch_sensors.read, 0, "==", 0, "TRIANGLE TOUCH")
            motors.run_until(evac_state.SPEED_FAST, evac_state.SPEED_FAST, touch_sensors.read, 1, "==", 0, "TRIANGLE TOUCH")
            
            motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 2)
            motors.run(0, 0)
            
            # black_count, silver_count, x, search_type = locate(black_count, silver_count)
            for _ in range(0, 5): evac_camera.capture()
            motors.run(0, 0, 0.5)

            route_success = route(black_count, silver_count, None, search_type, retry=True)
            if not route_success: continue
            
            dump(search_type)
            
            motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.2)
            motors.run( evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 1)
            
        else:
            grab_success, prev_insert = grab(prev_insert)
            if not grab_success:
                motors.run(-evac_state.SPEED_FAST, -evac_state.SPEED_FAST, 0.7)
                continue
            
    print("LEAVING")
    
    motors.run(0, 0, 0.5)
    leave()
    
    motors.run(0, 0)
    print("EXITED!")
            
##########################################################################################################################################################
##########################################################################################################################################################
##########################################################################################################################################################

if __name__ == "__main__":
    # evac_state.victim_count = 3
    
    main()
    
    # leave()