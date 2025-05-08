from core.shared_imports import os, sys, time, randint, Optional, operator, cv2, np
from core.utilities import debug
from hardware.robot import *
from main import display_manager

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

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
        
class LineFollower():
    def __init__(self):
        self.straight_speed = 30
        self.turn_multi = 1.5
        self.min_black_area = 4000
        self.min_green_area = 1000
        self.base_black = 50
        self.light_black = 60
        self.lightest_black = 100

        self.last_angle = 90
        self.angle = 90
        self.image = None
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
        image = camera.capture_array()
        
        if self.__timing: t1 = time.perf_counter()
        display_image = image.copy()

        self.find_black()
        if self.__timing: t2 = time.perf_counter()

        self.find_green()
        if self.__timing: t3 = time.perf_counter()

        self.green_check()
        if self.__timing: t4 = time.perf_counter()

        self.calculate_angle(self.black_contour)
        if self.__timing: t5 = time.perf_counter()
        
        self.__turn()

        if display_image is not None and camera.X11:
            small_image = cv2.resize(display_image, (0, 0), fx=0.5, fy=0.5)
            display_manager.show(np.uint8(small_image), name="line")

        if self.__timing: t6 = time.perf_counter()

        elapsed_time = time.perf_counter() - start_time
        fps = int(1.0 / elapsed_time) if elapsed_time > 0 else 0

        # print(f"[TIMING] capture={t1-t0:.3f}s black={t2-t1:.3f}s green={t3-t2:.3f}s check={t4-t3:.3f}s angle={t5-t4:.3f}s display={t6-t5:.3f}s total={elapsed_time:.3f}s")
        return fps, self.error, self.green_signal

    def __turn(self):
        if self.green_signal == "Double":
            self.angle = 90
            self.error = 0
        else:
            self.error = int(self.turn_multi * (self.angle - 90))
            if abs(self.error) < 10:
                self.error = 0

        if self.green_signal == "Double":
            motors.run(40, -40, 2)
            motors.run(30, -30)

            while True:
                self.image = camera.capture_array()
                self.display_image = self.image.copy()

                self.find_black()
                self.calculate_angle(self.black_contour)

                if self.display_image is not None and camera.X11:
                    small_image = cv2.resize(self.display_image, (0, 0), fx=0.5, fy=0.5)
                    display_manager.show(np.uint8(small_image), name="line")
                
                if self.angle < 90+20 and self.angle > 90-20 and self.angle != 90: break
            
        else:
            v1 = self.straight_speed + self.error
            v2 = self.straight_speed - self.error
            if robot_state.trigger["uphill"]:
                v1 = self.straight_speed + 10 + self.error * 0.5
                v2 = self.straight_speed + 10 - self.error * 0.5

            # if robot_state.trigger["tilt_right"]:
            #     v1 = 15 + self.error
            #     v2 = 15 - self.error
            #     if self.error < 0: v1 = 0
            #     elif self.error > 0: v2 = 0
            # elif robot_state.trigger["uphill"]:
            #     if self.error < 0: v1 = 5
            #     elif self.error > 0: v2 = 5

            self.motors.run(v1, v2)
    
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
        x_check = int((top_left[0] + top_right[0]) / 2)
        y_check = int((top_left[1] + top_right[1]) / 2) - 10

        # Check if point is within image bounds
        if 0 <= x_check < camera.LINE_WIDTH and 0 <= y_check < camera.LINE_HEIGHT and self.black_mask is not None:
            if self.black_check((x_check, y_check)):
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
                cv2.circle(self.display_image, check_point, 2*check_size, (0, 255, 255), -1)

            # Check if any pixel in the region is black
            if np.any(region == 255):
                # Check if green is present in the region
                if self.green_contours is not None:
                    for contour in self.green_contours:
                        # Iterate over pixels in the region
                        for y in range(y_start, y_end):
                            for x in range(x_start, x_end):
                                if self.black_mask[y, x] == 255:
                                    if cv2.pointPolygonTest(contour, (x, y), False) >= 0:
                                        # Found green at black pixel → fail
                                        return False
                    
            return np.any(region == 255)
        return False

    def green_hold(self):
        if self.green_signal != None and self.green_signal != "Double" and self.prev_green_signal != self.green_signal:
            self.last_seen_green = time.perf_counter()
        elif self.green_signal == "Double":
            self.last_seen_green = 0

        if time.perf_counter() - self.last_seen_green < 0.3:
            if self.prev_green_signal != "Double" and self.prev_green_signal is not None:
                print("Holding green")
                self.green_signal = self.prev_green_signal

    def find_green(self):
        self.green_contours = []
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_image, (40, 50, 50), (90, 255, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        green_mask = cv2.erode(green_mask, kernel, iterations=2)
        contours, _ = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        green_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_green_area]
        if robot_state.trigger["tilt_right"] or robot_state.trigger["tilt_left"]:
            self.green_contours = green_contours
        else:
            for contour in green_contours:
                if all(p[0][1] > int(self.camera.LINE_HEIGHT/3) for p in contour) and contour is not None and len(contour) > 0:
                    self.green_contours.append(contour)

        if self.green_contours and self.display_image is not None and camera.X11:
            cv2.drawContours(self.display_image, self.green_contours, -1, (0, 0, 255), 2)


    # BLACK
    def calculate_angle(self, contour=None, validate=False):
        ref_point = None
        self.angle = 90
        if contour is not None:
            ref_point = self.calculate_top_contour(contour, validate)

            left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= 10]
            right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= camera.LINE_WIDTH - 10]

            if self.green_signal == "Left":     
                point = min(contour, key=lambda p: p[0][0], default=None)
                ref_point = tuple(point[0])
            elif self.green_signal == "Right":  
                point = max(contour, key=lambda p: p[0][0], default=None)
                ref_point = tuple(point[0])
            elif ref_point[1] > 10 and (left_edge_points or right_edge_points):
                y_avg_left = int(np.mean([p[1] for p in left_edge_points])) if left_edge_points else None
                y_avg_right = int(np.mean([p[1] for p in right_edge_points])) if right_edge_points else None

                if y_avg_left is not None and (self.last_angle < 90 or y_avg_right is None or y_avg_left < y_avg_right):
                    ref_point = (0, y_avg_left)
                elif y_avg_right is not None:
                    ref_point = (camera.LINE_WIDTH - 1, y_avg_right)
                
        if ref_point is not None:
            bottom_center = (camera.LINE_WIDTH // 2, camera.LINE_HEIGHT)
            dx = bottom_center[0] - ref_point[0]
            dy = bottom_center[1] - ref_point[1]

            # Calculate angle in degrees
            angle_radians = np.arctan2(dy, dx)
            angle = int(np.degrees(angle_radians))
            if validate == False:
                self.angle = angle
                self.last_angle = angle
            else:
                return angle

            if self.display_image is not None and camera.X11:
                cv2.line(self.display_image, ref_point, bottom_center, (0, 0, 255), 2)
    
        return 90
        
    def calculate_top_contour(self, contour, validate):
        ys = [pt[0][1] for pt in contour]
        min_y = min(ys)

        # build your threshold—only 2px below the top
        threshold_y = min_y + 20

        # select just those points
        top_points = [pt for pt in contour if pt[0][1] <= threshold_y]
        if not top_points:
            return None

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
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Threshold masks for different brightness levels
        base_mask = cv2.inRange(gray, 0, self.base_black)
        light_mask = cv2.inRange(gray, 0, self.light_black)
        lightest_mask = cv2.inRange(gray, 0, self.lightest_black)
        # Create region masks
        region_mask_lightest = np.zeros_like(gray, dtype=np.uint8)
        region_mask_light = np.zeros_like(gray, dtype=np.uint8)
        region_mask_base = np.ones_like(gray, dtype=np.uint8) * 255

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
                    if highest_point[0][1] > camera.LINE_HEIGHT / 2:
                        tallest_contours.append(contour)
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
            contour_mask = np.zeros(gray.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [self.black_contour], -1, color=255, thickness=cv2.FILLED)
            self.black_mask = contour_mask

            if camera.X11:
                for c in contours:
                    color = (255, 255, 0) if c is self.black_contour else (255, 0, 0)
                    cv2.drawContours(self.display_image, [c], -1, color, 2)

def main() -> None:
    robot_state = RobotState()
    line_follow = LineFollower()
    led.on()
    
    while True:            
        fps = error = 0
        green_signal = None

        colour_values = colour_sensors.read()
        touch_values = touch_sensors.read()
        gyro_values = gyroscope.read()
        
        touch_check(robot_state, touch_values)
        ramp_check(robot_state, gyro_values)
        
        # red_check(robot_state)
        # silver_check(robot_state, colour_values)

        # if robot_state.count["red"] >= 1:
        #     # Found red stop
        #     robot_state.count["red"] = 0
            
        #     motors.run(config.line_speed, config.line_speed, 0.5)
        #     motors.run(0, 0, 10)
            
        # elif robot_state.count["silver"] >= 20 and evacuation_zone_enabled:
        #     # Found evacuation zone
        #     evacuation_zone.main()
            
        #     # Line follow with camera
        #     robot_state.trigger["evacuation_zone"] = True
        
    #     # Line follow with camera
    #     robot_state.trigger["evacuation_zone"] = True

        if robot_state.count["touch"] > 10:
            robot_state.count["touch"] = 0
            avoid_obstacle(line_follow)
        else:
            # Line Follow
            led.on()
            fps, error, green_signal = line_follow.follow_line()

        active_triggers = ["LINE"]
        for key in robot_state.trigger:
            if robot_state.trigger[key]: active_triggers.append(key)

        debug([f" ".join(active_triggers), str(robot_state.main_loop_count), str(fps), str(error), str(green_signal)], [10, 15, 10, 10, 15])

        # Update the loop count
        robot_state.main_loop_count += 1

def touch_check(robot_state: RobotState, touch_values: list[int]) -> None:
    robot_state.count["touch"] = robot_state.count["touch"] + 1 if sum(touch_values) != 2 else 0
     
def ramp_check(robot_state: RobotState, gyro_values: list[int]) -> None:
    if gyro_values is not None:
        pitch = gyro_values[0]
        roll  = gyro_values[1]
        
        robot_state.count["uphill"]     = robot_state.count["uphill"]     + 1 if pitch >=  15 else 0
        robot_state.count["downhill"]   = robot_state.count["downhill"]   + 1 if pitch <= -15 else 0
        
        robot_state.count["tilt_left"]  = robot_state.count["tilt_left"]  + 1 if roll  <= -15 else 0
        robot_state.count["tilt_right"] = robot_state.count["tilt_right"] + 1 if roll  >= 15 else 0
        
        robot_state.last_uphill = 0 if robot_state.trigger["uphill"] else robot_state.last_uphill + 1

# def red_check(robot_state: cRobotState) -> None:
#     # Only process images every 10 loops
#     # Only process images if we are not mid-turn
#     # Only process images if we are not using camera

#     if (   robot_state.main_loop_count % 10 != 0
#         or robot_state.pid_values
#         or not camera_enable                     ):

#         robot_state.count["red"] = 0
#         return None

#     image = camera.capture_array()
#     image = camera.perspective_transform(image, "LINE")
#     if config.X11: cv2.imshow("image", image)

#     hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
#     mask_lower = cv2.inRange(hsv_image, (0, 230, 46), (10, 255, 255))
#     mask_upper = cv2.inRange(hsv_image, (170, 230, 0), (179, 255, 255))
#     mask = cv2.bitwise_or(mask_lower, mask_upper)

#     mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)

#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     if not contours:
#         robot_state.count["red"] = 0
#         return None

#     # Find valid contours based on criteria
#     valid_contours = []

#     for contour in contours:
#         _, _, w, h = cv2.boundingRect(largest_contour)

#         if (   cv2.contourArea(contour) >= 100
#             and w > h                         ):
#             valid_contours.append(contour)

#     # Find the largest contour
#     largest_contour = max(valid_contours, key=cv2.contourArea)

#     if not largest_contour:
#         robot_state.count["red"] = 0
#         return None
#     else:
#         robot_state.count["red"] += 1

# def silver_check(robot_state: cRobotState, colour_values: list[int]) -> None:
#     global silver_min

#     if robot_state.pid_values["ir"]["integral"] >= 1000:
#         robot_state.count["silver"] = 0
#         return None

#     if any(colour_values[i] > silver_min for i in [0, 1, 3, 4]) and colour_values[2] > 60:
#         robot_state.count["silver"] += 1
#     else:
#         robot_state.count["silver"] = 0

#     return silver_count
 
def update_triggers(robot_state: RobotState) -> list[str]:
    prev_triggers = robot_state.trigger
     
    # Counting modifiers
    if robot_state.count["uphill"] > 30:
        robot_state.trigger["uphill"] = True
    else:
        robot_state.trigger["uphill"] = False
         
    if robot_state.count["downhill"] > 0:
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
    
     
    # Seasaw modifier
    if robot_state.last_uphill <= 40 and robot_state.trigger["downhill"]:
        robot_state.trigger["downhill"] = False
        robot_state.trigger["seasaw"] = True
    else:
        robot_state.trigger["seasaw"] = False
     
    # If there has been a change in modifiers
    if robot_state.trigger != prev_triggers:
        # Check if the current trigger has been the same for AT LEAST 10 loops
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
    for i in range(25):
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
        if colour_values[colour_pin] <= 30 and time.perf_counter() - start_time > 0.5:
            break

        if laser_value is not None:
            error = max(min(int(wall_multi * (laser_value - target_distance)), int(1*line_follow.straight_speed)), int(-1*line_follow.straight_speed))
        else:
            error = 0

        if sum(touch_values) < 2:
            # motors.run(-v1, -v2, 0.3)
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
    
    # Turn in the opposite direction
    motors.run(-v1, -v2, 2.5)

def circle_obstacle(v1: float, v2: float, laser_pin: int, colour_pin: int, comparison: str, target_distance: float, text: str = "") -> bool:
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