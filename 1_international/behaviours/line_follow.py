from core.shared_imports import os, sys, time, randint, Optional, operator, cv2, np
from core.utilities import *
from hardware.robot import *
# from main import display_manager

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

start_display()

class RobotState():
    def __init__(self):
        self.main_loop_count = 0
        
        self.count = {
            "uphill": 0,
            "downhill": 0,
            "tilt_left": 0,
            "tilt_right": 0,
            "red": 0,
            "silver": 0,
            "touch": 0
        }
         
        self.trigger = {
            "uphill": False,
            "downhill": False,
            "tilt_left": False,
            "tilt_right": False,
            "seasaw": False,
            "evacuation_zone": False
        }
         
        self.last_uphill = 0
        self.last_downhill = 1000
        
class LineFollower():
    def __init__(self):
        self.straight_speed = 30
        self.turn_mutli = 1.5

        self.min_black_area = 1000
        self.min_green_area = 3000
        self.base_black = 50
        self.light_black = 80
        self.lightest_black = 100
        self.silver = 253
        
        self.turn_color = (0, 255, 0)
        self.prev_side = None
        self.last_angle = 90
        self.angle = 90
        self.image = None
        self.hsv_image = None
        self.gray_image = None
        self.display_image = None
        self.green_contours = None
        self.green_signal = None
        self.prev_green_signal = None
        self.last_seen_green = 100
        self.black_contour = None
        self.black_mask = None
        self.__timing = False
    
    def follow(self) -> None:
        start_time = time.perf_counter()

        if self.__timing: t0 = time.perf_counter()
        self.image = camera.capture_array()
        
        if self.__timing: t1 = time.perf_counter()
        self.display_image = self.image.copy()

        self.find_black()
        if self.__timing: t2 = time.perf_counter()

        self.find_green()
        if self.__timing: t3 = time.perf_counter()

        self.green_check()
        if self.__timing: t4 = time.perf_counter()

        self.calculate_angle(self.black_contour)
        if self.__timing: t5 = time.perf_counter()
                
        self.__turn()

        if self.display_image is not None and camera.X11:
            show(np.uint8(self.display_image), name="line")

        if self.__timing: t6 = time.perf_counter()

        elapsed_time = time.perf_counter() - start_time
        fps = int(1.0 / elapsed_time) if elapsed_time > 0 else 0

        if self.__timing:
            print(f"[TIMING] capture={t1-t0:.3f}s black={t2-t1:.3f}s green={t3-t2:.3f}s check={t4-t3:.3f}s angle={t5-t4:.3f}s display={t6-t5:.3f}s total={elapsed_time:.3f}s")
        
        return fps, self.turn, self.green_signal

    def __turn(self):
        if self.green_signal == "Double":
            self.angle = 90
            self.turn = 0
        else:
            if abs(90-self.angle) < 5:
                self.turn = 0
            else:
                self.turn = int(self.turn_multi * (self.angle - 90))

        if robot_state.trigger["seasaw"]:
            for i in range(30):
                motors.run(-30+i, -30+i, 0.04)
        elif self.green_signal == "Double":
            v1, v2, t = 40, -40, 2.5
            f = b = f_after = b_after = 0

            if robot_state.trigger["tilt_left"]:
                v1, v2, t = 50, -20, 3
                f = 0.5 if robot_state.trigger["uphill"] else 0
            elif robot_state.trigger["tilt_right"]:
                v1, v2, t = -20, 50, 3,
                f = 0.5 if robot_state.trigger["uphill"] else 0
            elif robot_state.trigger["uphill"]:
                f, t = 1.3, 3
            if robot_state.trigger["downhill"]:
                v1 -= 20
                v2 += 20
                t = 4
                b = 1

            motors.run(-self.straight_speed, -self.straight_speed, b)
            motors.run(self.straight_speed, self.straight_speed, f)
            motors.run(v1, v2, t)
            self.run_till_camera(v1, v2, 10)
        else:
            v1 = self.straight_speed + self.turn
            v2 = self.straight_speed - self.turn
            if robot_state.last_downhill < 100:
                v1 = self.straight_speed + self.turn * 0.7 - 20
                v2 = self.straight_speed - self.turn * 0.7 - 20
            elif robot_state.trigger["uphill"]:
                v1 += 10
                v2 += 10

            motors.run(v1, v2)
    
    def run_till_camera(self, v1, v2, threshold):
        motors.run(v1, v2)
        self.angle = 90
        while True:
            self.image = camera.capture_array()
            self.display_image = self.image.copy()

            self.find_black()
            self.calculate_angle(self.black_contour)

            if self.display_image is not None and camera.X11:
                show(np.uint8(self.display_image), name="line")
            
            if self.angle < 90+threshold and self.angle > 90-threshold and self.angle != 90: break
        motors.run(0, 0)

    # GREEN
    def green_check(self):
        self.green_signal = None

        if self.green_contours is not None:
            valid_rects = []
            for contour in self.green_contours:
                valid_rect = self.validate_green_contour(contour)
                if valid_rect is not None:
                    valid_rects.append(valid_rect)
            
            if len(valid_rects) > 1:
                self.green_signal = "Double"
            elif len(valid_rects) == 1:
                valid_rect = valid_rects[0]
                
                # Take 2 Leftmost points
                left_points = sorted(valid_rect, key=lambda p: p[0])[:2]
                # Average left points
                left_x = int(sum(p[0] for p in left_points) / len(left_points))
                left_y = int(sum(p[1] for p in left_points) / len(left_points))

                # Check black on left
                check_point = (left_x - 20, left_y)
                if self.black_check(check_point):
                    self.green_signal = "Right"
                else:
                    # Take 2 Rightmost points
                    right_points = sorted(valid_rect, key=lambda p: p[0])[-2:]
                    # Average right points
                    right_x = int(sum(p[0] for p in right_points) / len(right_points))
                    right_y = int(sum(p[1] for p in right_points) / len(right_points))

                    # Check black on right
                    check_point = (right_x + 20, right_y)
                    if self.black_check(check_point):
                        self.green_signal = "Left"

        self.green_hold()
        self.prev_green_signal = self.green_signal

    def validate_green_contour(self, contour):
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        y_sorted_green = sorted(box, key=lambda p: p[1])
        top_left, top_right = y_sorted_green[:2]
        top_left, top_right = tuple(top_left), tuple(top_right)

        # Average top left and top right points and increase y result
        x_avg = int((top_left[0] + top_right[0]) / 2)
        y_avg = int((top_left[1] + top_right[1]) / 2)
        if y_avg < int(camera.LINE_HEIGHT / 3):
            self.green_signal = "Approach"
            return None

        # Check if point is within image bounds
        if 0 <= x_avg < camera.LINE_WIDTH and 0 <= y_avg - 10 < camera.LINE_HEIGHT and self.black_mask is not None:
            if self.black_check((x_avg, y_avg - 10)):
                return box

        return None

    def black_check(self, check_point):
        check_size = 10

        if 0 <= check_point[0] < camera.LINE_WIDTH and 0 <= check_point[1] < camera.LINE_HEIGHT and self.black_mask is not None:
            # Create a region around the point
            y_start = max(0, check_point[1] - check_size)
            y_end = min(camera.LINE_HEIGHT, check_point[1] + check_size)
            x_start = max(0, check_point[0] - check_size)
            x_end = min(camera.LINE_WIDTH, check_point[0] + check_size)
            region = self.black_mask[y_start:y_end, x_start:x_end]

            # If display image, draw the check point
            if self.display_image is not None and camera.X11:
                cv2.circle(self.display_image, check_point, 2*check_size, (0, 255, 255), 2)

            # Check if any pixel in the region is black
            if not np.any(region == 255) or self.green_contours is None:
                return False

            for y in range(y_start, y_end):
                for x in range(x_start, x_end):
                    if self.black_mask[y, x] != 255:
                        continue
                        
                    in_any_contour = any(cv2.pointPolygonTest(contour, (x, y), False) >= 0 for contour in self.green_contours)
                    if not in_any_contour:
                        return True
        
        return False

    def green_hold(self):
        if self.green_signal != None and self.green_signal != "Double"  and self.green_signal != "Approach" and self.prev_green_signal != self.green_signal:
            self.last_seen_green = time.perf_counter()
        elif self.green_signal == "Double":
            self.last_seen_green = 0

        if time.perf_counter() - self.last_seen_green < 0.5 and self.prev_green_signal != "Double" and self.prev_green_signal != "Approach" and self.prev_green_signal is not None:
            self.green_signal = self.prev_green_signal

    def find_green(self):
        self.green_contours = []
        self.hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(self.hsv_image, (40, 50, 50), (90, 255, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        green_mask = cv2.erode(green_mask, kernel, iterations=2)
        green_mask = cv2.dilate(green_mask, kernel, iterations=10)
        contours, _ = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        green_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_green_area]
        self.green_contours = green_contours
                
        if self.green_contours and self.display_image is not None and camera.X11:
            cv2.drawContours(self.display_image, self.green_contours, -1, (255, 0, 255), 2)

    # BLACK
    def calculate_angle(self, contour=None, validate=False):
        # Early return for simple case
        if not validate:
            self.angle = 90
            if contour is None:
                return 90

        # Initialize ref_point
        ref_point = self.calculate_top_contour(contour, validate) if contour is not None else None

        # Handle green signal cases
        if contour is not None and self.green_signal in ["Left", "Right"]:
            self.prev_side = self.green_signal
            point = min(contour, key=lambda p: p[0][0]) if self.green_signal == "Left" else max(contour, key=lambda p: p[0][0])
            ref_point = tuple(point[0])
            return self._finalize_angle(ref_point, validate)

        # Edge detection and angle calculation
        if contour is not None and ref_point is not None:
            left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= 10]
            right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= camera.LINE_WIDTH - 10]

            # Handle edge cases
            if ref_point[1] < 30:
                self.prev_side = None
                return self._finalize_angle(ref_point, validate)

            if self.green_signal != "Approach" and ref_point[1] > 50 and (left_edge_points or right_edge_points):
                y_avg_left = int(np.mean([p[1] for p in left_edge_points])) if left_edge_points else None
                y_avg_right = int(np.mean([p[1] for p in right_edge_points])) if right_edge_points else None

                if self.prev_side is None:
                    self.prev_side = "Left" if self.angle < 90 else "Right" if self.angle > 90 else None

                if y_avg_left is not None and (y_avg_right is None or self.prev_side == "Left"):
                    ref_point = (0, y_avg_left)
                    self.prev_side = "Left"
                elif y_avg_right is not None:
                    ref_point = (camera.LINE_WIDTH - 1, y_avg_right)
                    self.prev_side = "Right"

    return self._finalize_angle(ref_point, validate) if ref_point is not None else 90

    def _finalize_angle(self, ref_point, validate):
        bottom_center = (camera.LINE_WIDTH // 2, camera.LINE_HEIGHT)
        dx = bottom_center[0] - ref_point[0]
        dy = bottom_center[1] - ref_point[1]

        angle_radians = np.arctan2(dy, dx)
        angle = int(np.degrees(angle_radians))

        if not validate:
            self.angle = angle
            self.last_angle = angle
            self._update_display(ref_point, angle)
        else:
            return angle

        return angle if validate else 90

    def _update_display(self, ref_point, angle):
        if self.display_image is None or not camera.X11:
            return

        deviation = abs(angle - 90)
        
        if deviation <= 5:
            self.turn_color = (0, 255, 0)
        elif deviation <= 20:
            ratio = (deviation - 5) / 15
            self.turn_color = (0, 255, int(255 * ratio))
        elif deviation <= 55:
            ratio = (deviation - 20) / 35
            self.turn_color = (0, int(255 * (1 - ratio)), 255)
        else:
            self.turn_color = (0, 0, 255)

        cv2.circle(self.display_image, ref_point, 10, self.turn_color, 2)
    
    def calculate_top_contour(self, contour, validate):
        ys = [pt[0][1] for pt in contour]
        if self.green_signal == "Approach":
            min_y = 40
            max_y = 80
        else:
            min_y = min(ys)
            max_y = min_y + 20

        top_points = [pt for pt in contour if min_y <= pt[0][1] <= max_y]
        if not top_points:
            return camera.LINE_WIDTH // 2, 0

        # debug‑draw
        if self.display_image is not None and validate is False and camera.X11:
            for pt in top_points:
                x, y = pt[0]
                cv2.circle(self.display_image, (x, y), radius=3, color=(0, 0, 255), thickness=-1)

        # average them
        x_avg = int(sum(pt[0][0] for pt in top_points) / len(top_points))
        y_avg = int(sum(pt[0][1] for pt in top_points) / len(top_points))
        return x_avg, y_avg
    
    def find_black(self):
        self.black_contour = None
        self.black_mask = None
        self.gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Threshold masks for different brightness levels
        base_mask = cv2.inRange(self.gray_image, 0, self.base_black)
        light_mask = cv2.inRange(self.gray_image, 0, self.light_black)
        lightest_mask = cv2.inRange(self.gray_image, 0, self.lightest_black)
        # Create region masks
        region_mask_lightest = np.zeros_like(self.gray_image, dtype=np.uint8)
        region_mask_light = np.zeros_like(self.gray_image, dtype=np.uint8)
        region_mask_base = np.ones_like(self.gray_image, dtype=np.uint8) * 255

        cv2.fillPoly(region_mask_lightest, [np.int32(camera.lightest_points)], 255)
        cv2.fillPoly(region_mask_light, [np.int32(camera.light_points)], 255)

        # Exclude those regions from the base region
        region_mask_base = cv2.subtract(region_mask_base, region_mask_lightest)
        region_mask_base = cv2.subtract(region_mask_base, region_mask_light)

        # Apply appropriate mask to each region
        masked_lightest = cv2.bitwise_and(lightest_mask, region_mask_lightest)
        masked_light = cv2.bitwise_and(light_mask, region_mask_light)
        masked_base = cv2.bitwise_and(base_mask, region_mask_base)

        # Combine them into the final black mask
        black_mask = cv2.bitwise_or(masked_lightest, masked_light)
        black_mask = cv2.bitwise_or(black_mask, masked_base)

        # Apply morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 
        # black_mask = cv2.erode(black_mask, kernel, iterations=5)
        black_mask = cv2.dilate(black_mask, kernel, iterations=2)

        # Contour detection
        contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_black_area]

        # Contour selection logic
        if len(contours) > 1:
            close_contours = []
            for contour in contours:
                angle = self.calculate_angle(contour, validate=True)
                if abs(angle - self.last_angle) < 15:
                    close_contours.append(contour)

            if len(close_contours) == 0:
                closest_contour = min(contours, key=lambda c: abs(self.calculate_angle(c, validate=True) - self.last_angle))
                self.black_contour = closest_contour

            tallest_contours = []
            if len(close_contours) > 1:
                for contour in close_contours:
                    highest_point = max(contour, key=lambda p: p[0][1])
                    if highest_point[0][1] > camera.LINE_HEIGHT / 2: tallest_contours.append(contour)
            elif len(close_contours) == 1:
                self.black_contour = close_contours[0]

            if len(tallest_contours) > 1:
                self.black_contour = max(tallest_contours, key=lambda c: cv2.contourArea(c))
            elif len(tallest_contours) == 1:
                self.black_contour = tallest_contours[0]

        elif len(contours) == 1:
            self.black_contour = contours[0]

        # Final mask drawing
        if self.black_contour is not None:
            contour_mask = np.zeros(self.gray_image.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [self.black_contour], -1, color=255, thickness=cv2.FILLED)
            self.black_mask = contour_mask

            if camera.X11:
                for c in contours:
                    if c is not self.black_contour: cv2.drawContours(self.display_image, [c], -1, (255, 0, 0), 2)
                    else: cv2.drawContours(self.display_image, [c], -1, self.turn_color, 2)
    
    # SILVER
    def find_silver(self):
        if self.gray_image is not None:
            mask = cv2.inRange(self.gray_image, self.silver, 255)
            silver_pixels = cv2.countNonZero(mask)
            total_pixels = self.gray_image.shape[0] * self.gray_image.shape[1]
            
            if silver_pixels / total_pixels >= 0.05:
                robot_state.count["silver"] += 1
                return

        robot_state.count["silver"] = 0
    
    # RED
    def find_red(self):
        if self.hsv_image is not None:
            mask = cv2.inRange(self.hsv_image, self.silver, 255)
            mask_lower = cv2.inRange(self.hsv_image, (0, 230, 46), (10, 255, 255))
            mask_upper = cv2.inRange(self.hsv_image, (170, 230, 0), (179, 255, 255))
            mask = cv2.bitwise_or(mask_lower, mask_upper)

            if cv2.countNonZero(mask) > 0:
                robot_state.count["red"] += 1
                return
        
        robot_state.count["red"] = 0

robot_state = RobotState()
line_follow = LineFollower()

def main(start_time) -> None:
    global robot_state, line_follow
    fps = error = 0
    green_signal = None

    colour_values = colour_sensors.read()
    touch_values = touch_sensors.read()
    gyro_values = gyroscope.read()
    touch_check(robot_state, touch_values)
    ramp_check(robot_state, gyro_values)
    update_triggers(robot_state)

    # red_check(robot_state)
    # if time.perf_counter() - start_time > 3:
        # line_follow.find_silver()
        # line_follow.find_red()

    if robot_state.count["silver"] >= 5:
        print("Silver Found!")
        motors.pause()
        robot_state.count["silver"] = 0
        evacuation_zone.main()
        robot_state.trigger["evacuation_zone"] = True
    elif robot_state.count["red"] >= 10:
        print("Red Found!")
        motors.run(0, 0, 8)
        robot_state.count["red"] = 0
    elif robot_state.count["touch"] > 5:
        robot_state.count["touch"] = 0
        avoid_obstacle(line_follow)
    else:
        led.on()
        fps, error, green_signal = line_follow.follow()

        active_triggers = ["LINE"]
        for key in robot_state.trigger:
            if robot_state.trigger[key]: active_triggers.append(key)

        debug([f" ".join(active_triggers), str(robot_state.main_loop_count), str(fps), str(error), str(green_signal)], [10, 15, 10, 10, 15])

    robot_state.main_loop_count += 1

def touch_check(robot_state: RobotState, touch_values: list[int]) -> None:
    robot_state.count["touch"] = robot_state.count["touch"] + 1 if sum(touch_values) != 2 else 0
     
def ramp_check(robot_state: RobotState, gyro_values: list[int]) -> None:
    if gyro_values is not None:
        pitch = gyro_values[0]
        roll  = gyro_values[1]
        
        robot_state.count["uphill"]     = robot_state.count["uphill"]     + 1 if pitch >=  10 else 0
        robot_state.count["downhill"]   = robot_state.count["downhill"]   + 1 if pitch <= -10 else 0
        
        robot_state.count["tilt_left"]  = robot_state.count["tilt_left"]  + 1 if roll  <= -10 else 0
        robot_state.count["tilt_right"] = robot_state.count["tilt_right"] + 1 if roll  >= 10 else 0
        
        robot_state.last_uphill = 0 if robot_state.trigger["uphill"] else robot_state.last_uphill + 1
        robot_state.last_downhill = 0 if robot_state.count["downhill"] > 0 else robot_state.last_downhill + 1
 
def update_triggers(robot_state: RobotState) -> list[str]:
    prev_triggers = robot_state.trigger

    if robot_state.count["uphill"] > 10:
        robot_state.trigger["uphill"] = True
    else:
        robot_state.trigger["uphill"] = False
         
    if robot_state.count["downhill"] > 10:
        robot_state.trigger["downhill"] = True
    else:
        robot_state.trigger["downhill"] = False
         
    if robot_state.count["tilt_left"] > 0:
        robot_state.trigger["tilt_left"] = True
    else:
        robot_state.trigger["tilt_left"] = False
         
    if robot_state.count["tilt_right"] > 0:
        robot_state.trigger["tilt_right"] = True
    else:
        robot_state.trigger["tilt_right"] = False
    
    if robot_state.last_uphill <= 20 and robot_state.count["downhill"] > 0:
        robot_state.trigger["downhill"] = False
        robot_state.trigger["seasaw"] = True
        robot_state.last_uphill = 100
    else:
        robot_state.trigger["seasaw"] = False

    if robot_state.trigger != prev_triggers:
        if robot_state.main_loop_count <= 10:
            robot_state.triggers = prev_triggers

def avoid_obstacle(line_follow: LineFollower) -> None:    
    while True:
        left_value = laser_sensors.read([0])[0]
        if left_value is not None: break
    
    while True:
        right_value = laser_sensors.read([1])[0]
        if right_value is not None: break

    side_values = [left_value, right_value]

    # Immediately update OLED for obstacle detection
    # oled_display.reset()
    # oled_display.text("Obstacle Detected", 0, 0, size=10)

    debug(["OBSTACLE", "FINDING SIDE", ", ".join(list(map(str, side_values)))], [24, 50, 14])
    
    # Clockwise if left > right, else random
    if side_values[0] <= 30 or side_values[1] <= 30:
        direction = "cw" if side_values[0] > side_values[1] else "ccw"
    else:
        direction = "cw" if randint(0, 1) == 0 else "ccw"

    # Turn until appropriate laser sees obstacle
    v1 = v2 = laser_pin = 0
    if direction == "cw":
        v1 = -line_follow.straight_speed
        v2 =  line_follow.straight_speed * 0.8
        laser_pin = 1
    else:
        v1 =  line_follow.straight_speed * 0.8
        v2 = -line_follow.straight_speed
        laser_pin = 0

    # SETUP
    # Over turn passed obstacle
    # oled_display.text("Turning till obstacle", 0, 10, size=10)
    motors.run(v1, v2, 1)
    for i in range(50):
        motors.run_until(1.2 * v1, 1.2*v2, laser_sensors.read, laser_pin, "<=", 20, "TURNING TILL OBSTACLE")
        motors.run(1.2 * v1, 1.2 * v2, 0.01)
    
    # oled_display.text("Turning past obstacle", 0, 20, size=10)
    for i in range(25):
        motors.run_until(1.2 * v1, 1.2*v2, laser_sensors.read, laser_pin, ">=", 20, "TURNING PAST OBSTACLE")
        motors.run(1.2 * v1, 1.2 * v2, 0.01)

    # Turn back onto obstacle
    # oled_display.text("Turning till obstacle", 0, 30, size=10)
    motors.run_until(-v1, -v2, laser_sensors.read, laser_pin, "<=", 15, "TURNING TILL OBSTACLE")
    motors.run(-1.2 * v1, -1.2 * v2, 1)

    # Circle obstacle
    if direction == "cw":
        v1 =  line_follow.straight_speed
        v2 = -line_follow.straight_speed
        laser_pin = 1
        colour_pin = 2
    else:
        v1 = -line_follow.straight_speed
        v2 =  line_follow.straight_speed
        laser_pin = 0
        colour_pin = 2

    # oled_display.text("Circle Obstacle: Starting", 0, 40, size=10)
    circle_obstacle(line_follow.straight_speed, line_follow.straight_speed, laser_pin, colour_pin, "<=", 13, "FORWARDS TILL OBSTACLE")

    motors.run(0, 0, 0.15)

    start_time = time.perf_counter()
    wall_multi = 8
    target_distance = 1

    while True:
        laser_value = laser_sensors.read([laser_pin])[0]
        colour_values = colour_sensors.read()
        touch_values = touch_sensors.read()
        if colour_values[colour_pin] <= 30 and time.perf_counter() - start_time > 1.5:
            break

        if laser_value is not None:
            error = max(min(int(wall_multi * (laser_value - target_distance)), int(1*line_follow.straight_speed)), int(-1*line_follow.straight_speed))
            if laser_value > 20:
                motors.run(line_follow.straight_speed, line_follow.straight_speed, 0.2)
        else:
            error = 0

        if sum(touch_values) < 2:
            motors.run_until(-v1, -v2, laser_sensors.read, laser_pin, "<=", 10, "TURNING BACK TILL OBSTACLE")
            motors.run(line_follow.straight_speed, line_follow.straight_speed, 0.15)

        print(error)
        if direction == "cw":
            motors.run(line_follow.straight_speed + error, line_follow.straight_speed - error)
        else:
            motors.run(line_follow.straight_speed - error, line_follow.straight_speed + error)

    # oled_display.reset()
    # oled_display.text("Black Found", 0, 0, size=10)
    debug(["OBSTACLE", "FOUND BLACK"], [24, 50])
    
    line_follow.run_till_camera(-v1, -v2, 30)

def circle_obstacle(v1: float, v2: float, laser_pin: int, colour_pin: int, comparison: str, target_distance: float, text: str = "") -> bool:
    global camera
    if   comparison == "<=": comparison_function = operator.le
    elif comparison == ">=": comparison_function = operator.ge

    while True:
        laser_value = laser_sensors.read([laser_pin])[0]
        colour_values = colour_sensors.read()

        debug(["OBSTACLE", text, f"{laser_value}"], [24, 50, 10])

        motors.run(v1, v2)
        if laser_value is not None:
            if comparison_function(laser_value, target_distance) and laser_value != 0: return True

        if colour_values[colour_pin] <= 30:
            return False

"""
TESTING
"""

if __name__ == "__main__": pass