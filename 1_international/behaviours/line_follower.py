from core.shared_imports import time, cv2, np
from core.utilities import show
from core.listener import listener
from hardware.robot import *
        
class LineFollower():
    def __init__(self, robot_state):
        # Constants
        self.robot_state = robot_state
        self.speed = 30
        self.turn_multi = 1.8
        self.integral_multi = 0.008
        self.min_black_area = 5000
        self.min_green_area = 3000
        self.base_black = 80
        self.light_black = 130
        self.lightest_black = 140
        self.turn_color = (255,255, 0)
        self.lower_blue = np.array([100, 100, 100])
        self.upper_blue = np.array([130, 255, 255])

        # Variables
        self.prev_side = None
        self.last_angle = self.angle = 90
        self.turn = self.v1 = self.v2 = self.integral = 0
        self.image = self.hsv_image = self.prev_gray = self.gray_image = self.display_image = self.blue_contours = self.green_contours = self.green_signal = self.prev_green_signal = self.black_contour = self.black_mask = None 
        self.last_seen_green = 100
    
    # ========================================================================
    # MAIN LOOP
    # ========================================================================

    def follow(self, starting=False) -> None:
        self.image = camera.perspective_transform(camera.capture_array())
        self.display_image = self.image.copy()

        self.find_black()
        if self.black_contour is not None:
            self.find_green()
            self.green_check()
            self.calculate_angle(self.black_contour)
            if not starting: self.__turn()

        elif not starting: 
            oled_display.text("GAP", 38, 12, size=30, clear=True)
            self.gap_handling()

        if self.display_image is not None and camera.X11: 
            show(self.display_image, display=camera.X11, name="line", debug_lines=self.robot_state.debug_text)

    # ========================================================================
    # CALCULATE TURN
    # ========================================================================

    def __turn(self):
        # Determining Angle and Integral
        if self.green_signal == "DOUBLE":
            self.angle = 90
            self.turn = 0
        else:
            if abs(90-self.angle) < 5:
                self.turn = 0
            else:
                self.turn = int(self.turn_multi * (self.angle - 90))
                self.integral += self.angle-90
                self.turn = self.turn + int(self.integral * self.integral_multi) if self.robot_state.last_downhill < 200 or self.robot_state.last_uphill < 200 else self.turn
            
            if abs(90-self.angle) < 10:
                self.integral = 0

            self.robot_state.debug_text.append(f"TURN: {self.turn}")
            self.robot_state.debug_text.append(f"INT: {self.integral}")

        # Seesaw
        if self.robot_state.trigger["seasaw"]:
            oled_display.text("SEESAW", 8, 12, size=30, clear=True)
            for i in range(5): motors.run(-30+i*6, -30+i*6, 0.20)
            self.robot_state.debug_text.append(f"SEESAW")
        
        # DOUBLE Green
        elif self.green_signal == "DOUBLE":
            v1, v2, t = 35, -35, 3.8
            f = b = 0

            if self.robot_state.trigger["tilt_left"]:
                v1, v2, t = 40, -15, 3
                f = 0.5 if self.robot_state.trigger["uphill"] else 0
            elif self.robot_state.trigger["tilt_right"]:
                v1, v2, t = -15, 40, 3
                f = 0.5 if self.robot_state.trigger["uphill"] else 0
            elif self.robot_state.trigger["uphill"]:
                f, t = 1.3, 3
                v1 = v1
                v2 = v2 * 1.5
            if self.robot_state.trigger["downhill"]:
                v1 -= 5
                v2 += 5
                t -= 0.3
                b = 1

            motors.run(-30, -30, b)
            motors.run(30, 30, f)
            motors.run(v1, v2, t)
            motors.run(0, 0, 0.5)
            self.run_till_camera(v1, v2, 15, ["DOUBLE GREEN"])

        # Other Turns
        else:
            v1 = self.speed + self.turn
            v2 = self.speed - self.turn

            if self.robot_state.last_downhill < 200:
                speed = 18
                v1 = speed + self.turn
                v2 = speed - self.turn
                v1 = min(v1, max(-v2//5, speed))
                v2 = min(v2, max(-(speed + self.turn)//5, speed))

            elif self.robot_state.trigger["uphill"]:
                v1 += 5
                v2 += 5
                if v1 < 0: v1 = max(-30, v1)
                if v2 < 0: v2 = max(-30, v2)

            if self.robot_state.trigger["tilt_left"] and v2 < 0: v2 = v2 // 2

            if self.robot_state.trigger["tilt_right"] and v1 < 0: v1 = v1 // 2

            self.v1, self.v2 = v1, v2

            if self.stuck_check():                  oled_display.text("STUCK", 20, 12, size=30, clear=True),
            elif self.green_signal == "APPROACH":   oled_display.text("GREEN", 15, 0, size=30, clear=True)
            elif self.green_signal == "LEFT":       oled_display.text(self.green_signal, 25 ,30, size=30, clear=False)
            elif self.green_signal == "RIGHT":      oled_display.text(self.green_signal, 15, 30, size=30, clear=False)
            elif self.green_signal == "DOUBLE":     oled_display.text(self.green_signal, 8, 30, size=30, clear=False)
            else:                                   oled_display.text("LINE", 25, 12, size=30, clear=True)
                    
            motors.run(self.v1, self.v2)
    
    # ========================================================================
    # HELPERS
    # ========================================================================
    
    def run_till_camera(self, v1, v2, threshold, text: list[str], obstacle: bool = False):
        motors.run(v1, v2)
        self.angle = 90
        
        while listener.mode.value != 0:
            self.image = camera.perspective_transform(camera.capture_array())
            self.display_image = self.image.copy()

            self.find_black(obstacle)
            self.calculate_angle(self.black_contour)

            if self.display_image is not None and camera.X11:
                show(self.display_image, display=camera.X11, name="line", debug_lines=text)
            
            if self.angle < 90+threshold and self.angle > 90-threshold and self.angle != 90: break
            
        motors.run(0, 0)

    def stuck_check(self) -> bool:
        if abs(self.integral) > 3000: 
            self.v1 += 15
            self.v2 += 15
            self.robot_state.debug_text.append(f"STUCK")
            return True
        return False
        

    # ========================================================================
    # GREEN
    # ========================================================================
    def green_check(self):
        self.green_signal = None

        if self.green_contours is not None:
            valid_rects = []
            for contour in self.green_contours:
                valid_rect = self.validate_green_contour(contour)
                if valid_rect is not None:
                    valid_rects.append(valid_rect)
            
            if len(valid_rects) > 1:
                self.green_signal = "DOUBLE"
            elif len(valid_rects) == 1:
                valid_rect = valid_rects[0]
                
                # Take 2 Leftmost points
                left_points = sorted(valid_rect, key=lambda p: p[0])[:2]
                # Average left points
                left_x = int(sum(p[0] for p in left_points) / len(left_points))
                left_y = int(sum(p[1] for p in left_points) / len(left_points))

                # Check black on left
                check_point = (left_x - int(camera.LINE_WIDTH / 16), left_y)
                if self.black_check(check_point):
                    self.green_signal = "RIGHT"
                else:
                    # Take 2 Rightmost points
                    right_points = sorted(valid_rect, key=lambda p: p[0])[-2:]
                    # Average right points
                    right_x = int(sum(p[0] for p in right_points) / len(right_points))
                    right_y = int(sum(p[1] for p in right_points) / len(right_points))

                    # Check black on right
                    check_point = (right_x + int(camera.LINE_WIDTH / 16), right_y)
                    if self.black_check(check_point):
                        self.green_signal = "LEFT"

        self.green_hold()
        self.prev_green_signal = self.green_signal
        self.robot_state.debug_text.append(f"GREEN: {self.green_signal}")

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
        # if y_avg < int(camera.LINE_HEIGHT / 3) and self.robot_state.trigger["downhill"] is False:
        if y_avg < int(camera.LINE_HEIGHT / 3):
            self.green_signal = "APPROACH"
            return None
        elif y_avg < int(camera.LINE_HEIGHT / 2) and self.robot_state.trigger["uphill"] is True:
            self.green_signal = "APPROACH"
            return None

        # Check if point is within image bounds
        if 0 <= x_avg < camera.LINE_WIDTH and 0 <= y_avg - int(camera.LINE_HEIGHT / 20) < camera.LINE_HEIGHT and self.black_mask is not None:
            if self.black_check((x_avg, y_avg - int(camera.LINE_HEIGHT / 20))):
                return box

        return None

    def black_check(self, check_point):
        check_size = int(camera.LINE_WIDTH / 32)

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
        if self.green_signal not in [None, "DOUBLE", "APPROACH"] and self.prev_green_signal != self.green_signal:
            self.last_seen_green = time.perf_counter()
        elif self.green_signal == "DOUBLE":
            self.last_seen_green = 0

        if time.perf_counter() - self.last_seen_green < 0.5 and self.prev_green_signal not in [None, "DOUBLE", "APPROACH"]:
            self.green_signal = self.prev_green_signal

    def find_green(self):
        self.green_contours = []
        self.hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(self.hsv_image, (40, 50, 50), (90, 255, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        green_mask = cv2.erode(green_mask, kernel, iterations=1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        green_mask = cv2.dilate(green_mask, kernel, iterations=2)
        contours, _ = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        green_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_green_area]
        self.green_contours = green_contours
                
        if self.green_contours and self.display_image is not None and camera.X11:
            cv2.drawContours(self.display_image, self.green_contours, -1, (255, 0, 255), int(camera.LINE_HEIGHT/100))

    # ========================================================================
    # GAP
    # ========================================================================

    def gap_handling(self):
        print("Gap Detected!")
        motors.run(0, 0, 1)
        motors.run(-25, -25)

        for i in range(3): self.__wait_for_black_contour()
        motors.run(-25, -25, 0.1)
        motors.run(0, 0, 0.3)

        self.align_to_contour_angle()
        motors.run(0, 0, 0.2)

        f = 1.1 + 0.5 * (self.robot_state.last_uphill < 100)
        if not self.__move_and_check_black(f):
            print("No black found, retrying...")

            motors.run(-35, -35, f-0.1)
            motors.run(0, 0, 0.2)

            self.align_to_contour_angle()
            motors.run(0, 0, 0.2)

            if not self.__move_and_check_black(f+1.1):
                print("Still no black. Exiting gap handler.")
                motors.run(-35, -35, f+1)

        print("Gap handling complete.")
    
    def __wait_for_black_contour(self):
        while listener.mode.value != 0:
            self.image = camera.perspective_transform(camera.capture_array())
            self.display_image = self.image.copy()
            self.find_black()

            if self.black_contour is not None:
                if any(p[0][1] >= camera.LINE_HEIGHT - 5 for p in self.black_contour) and cv2.contourArea(self.black_contour) > 2000:
                    print("Black contour found.")
                    break

            if self.display_image is not None and camera.X11:
                show(self.display_image, display=camera.X11, name="line", debug_lines=["GAP"])

    def align_to_contour_angle(self):
        while listener.mode.value != 0:
            self.image = camera.perspective_transform(camera.capture_array())
            self.display_image = self.image.copy()
            self.find_black()

            if self.black_contour is not None and self.black_mask is not None:
                mask = self.black_mask.copy()

                # remove everything above the bottom half of the screen if last uphill was recent
                if self.robot_state.last_uphill < 100:
                    mask[0:int(camera.LINE_HEIGHT/2), :] = 0

                # Re-find contour on the modified mask
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not contours:
                    print("No contours after top cutoff.")
                    return

                contour = max(contours, key=cv2.contourArea)

                # --- Approximate contour to a polygon ---
                epsilon = 0.01 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                approx_points = approx[:, 0, :]  # shape (N, 2)

                if len(approx_points) < 2:
                    print("Polygon approximation too small.")
                    continue

                # --- TOP: average of top 2 points with smallest Y ---
                top_two = sorted(approx_points, key=lambda p: p[1])[:2]
                top_point = (
                    int(np.mean([pt[0] for pt in top_two])),
                    int(np.mean([pt[1] for pt in top_two]))
                )

                # --- BOTTOM: mask-based average of bottom 2 rows ---
                y_start = camera.LINE_HEIGHT - 2
                y_end = camera.LINE_HEIGHT
                band_mask = np.zeros_like(self.black_mask)
                band_mask[y_start:y_end, :] = 255
                bottom_band = cv2.bitwise_and(self.black_mask, band_mask)
                ys, xs = np.where(bottom_band == 255)

                if len(xs) == 0:
                    print("No bottom pixels found.")
                    continue

                bottom_point = (int(np.mean(xs)), int(np.mean(ys)))

                # --- ANGLE from bottom to top ---
                dx = top_point[0] - bottom_point[0]
                dy = top_point[1] - bottom_point[1]
                angle_rad = np.arctan2(dy, dx)
                angle = int(np.degrees(angle_rad))
                if angle < 0:
                    angle += 180

                print(f"[Gap Align] Angle (Poly): {angle:.2f}")

                # --- Alignment logic ---
                if abs(angle - 90) < 4:
                    break
                elif angle > 90:
                    motors.run(14, -18)
                else:
                    motors.run(-18, 14)

                # --- DEBUG DRAW ---
                if self.display_image is not None and camera.X11:
                    # Draw poly approximation
                    cv2.polylines(self.display_image, [approx], isClosed=True, color=(255, 0, 255), thickness=int(camera.LINE_HEIGHT/100))

                    # Draw angle line
                    cv2.line(self.display_image, bottom_point, top_point, (0, 255, 0), int(camera.LINE_HEIGHT/100))
                    cv2.circle(self.display_image, top_point, 5, (0, 255, 255), int(camera.LINE_HEIGHT/100))
                    cv2.circle(self.display_image, bottom_point, 5, (255, 255, 0), int(camera.LINE_HEIGHT/100))

                    show(self.display_image, display=camera.X11, name="line", debug_lines=["GAP"])

    def __move_and_check_black(self, duration: float) -> bool:
        motors.run(35, 35, duration)
        motors.run(0, 0, 0.1)

        self.image = camera.perspective_transform(camera.capture_array())
        self.display_image = self.image.copy()
        self.find_black()

        if self.black_contour is not None:
            return True
        return False

    # ========================================================================
    # BLACK
    # ========================================================================

    def calculate_angle(self, contour=None, validate=False):
        if not validate:
            self.angle = 90
            if contour is None: return 90

        ref_point = self.calculate_top_contour(contour) if contour is not None else None

        if contour is not None and self.green_signal in ["LEFT", "RIGHT"]:
            self.prev_side = self.green_signal
            point = min(contour, key=lambda p: p[0][0]) if self.green_signal == "LEFT" else max(contour, key=lambda p: p[0][0])
            ref_point = tuple(point[0])
            return self._finalize_angle(ref_point, validate)

        if contour is not None and ref_point is not None:
            left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= int(camera.LINE_WIDTH/16)]
            right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= camera.LINE_WIDTH - int(camera.LINE_WIDTH/16)]

            if ref_point[1] < (int(camera.LINE_HEIGHT/4) + self.robot_state.trigger["uphill"] * (camera.LINE_HEIGHT // 2 - int(3*camera.LINE_HEIGHT/20)) + int(camera.LINE_HEIGHT/10) * (self.robot_state.last_downhill < 100)):
                return self._finalize_angle(ref_point, validate)

            if self.green_signal != "APPROACH" and (left_edge_points or right_edge_points):
                y_avg_left = int(np.mean([p[1] for p in left_edge_points])) if left_edge_points else None
                y_avg_right = int(np.mean([p[1] for p in right_edge_points])) if right_edge_points else None

                if self.prev_side is None:
                    if y_avg_left is None and y_avg_right is not None:
                        self.prev_side = "Right"
                    elif y_avg_right is None and y_avg_left is not None:
                        self.prev_side = "Left"
                    else:
                        self.prev_side = "Left" if y_avg_left < y_avg_right else "Right" if  y_avg_left > y_avg_right else None

                if y_avg_left is not None and (y_avg_right is None or self.prev_side == "Left"):
                    ref_point = (0, y_avg_left)
                    self.prev_side = "Left"
                elif y_avg_right is not None:
                    ref_point = (camera.LINE_WIDTH - 1, y_avg_right)
                    self.prev_side = "Right"

        return_val = self._finalize_angle(ref_point, validate) if ref_point is not None else 90
        return return_val

    def _finalize_angle(self, ref_point, validate):
        bottom_center = (camera.LINE_WIDTH // 2, camera.LINE_HEIGHT)
        dx = bottom_center[0] - ref_point[0]
        dy = bottom_center[1] - ref_point[1]

        angle_radians = np.arctan2(dy, dx)
        angle = int(np.degrees(angle_radians))

        if not validate:
            self.angle = angle
            self.last_angle = angle
            if ref_point is not None: cv2.circle(self.display_image, ref_point, int(camera.LINE_HEIGHT / 15), self.turn_color, 2)
        else:
            return angle

        return angle if validate else 90
    
    def calculate_top_contour(self, contour):
        if contour is None:
            return camera.LINE_WIDTH // 2, 0

        # Create mask from contour
        contour_mask = np.zeros_like(self.gray_image, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

        # Define scan band
        min_y = int(camera.LINE_HEIGHT/5) if self.green_signal == "APPROACH" else min(pt[0][1] for pt in contour)
        if self.robot_state.trigger["uphill"]:
            min_y = camera.LINE_HEIGHT // 2
        elif self.robot_state.trigger["downhill"]:
            min_y = int(camera.LINE_HEIGHT/4)
        max_y = min(camera.LINE_HEIGHT, min_y + int(camera.LINE_HEIGHT/20))

        roi_mask = np.zeros_like(self.gray_image, dtype=np.uint8)
        roi_mask[min_y:max_y, :] = 255
        masked = cv2.bitwise_and(contour_mask, roi_mask)

        # Extract (x, y) points
        ys, xs = np.where(masked == 255)
        if len(xs) == 0:
            return camera.LINE_WIDTH // 2, camera.LINE_HEIGHT

        points = list(zip(xs, ys))

        # Downsample with linspace
        max_points = 20
        if len(points) > max_points:
            idx = np.linspace(0, len(points) - 1, max_points, dtype=int)
            points = [points[i] for i in idx]

        # average for stability
        x_avg = int(np.mean([pt[0] for pt in points]))
        y_avg = int(np.mean([pt[1] for pt in points]))
        return x_avg, y_avg

    def find_black(self, extra_erode: bool = False): 
        self.black_contour = None
        self.black_mask = None
        self.gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Threshold masks for different brightness levels
        base_mask     = cv2.inRange(self.gray_image, 0,     self.base_black)
        light_mask    = cv2.inRange(self.gray_image, 0,    self.light_black)
        lightest_mask = cv2.inRange(self.gray_image, 0, self.lightest_black)

        # Create region masks
        region_mask_lightest = np.zeros_like(self.gray_image, dtype=np.uint8)
        region_mask_light_left = np.zeros_like(self.gray_image, dtype=np.uint8)
        region_mask_light_right = np.zeros_like(self.gray_image, dtype=np.uint8)
        region_mask_base     = np.ones_like(self.gray_image, dtype=np.uint8) * 255

        cv2.fillPoly(region_mask_lightest, [np.int32(camera.lightest_points)], 255)
        cv2.fillPoly(region_mask_light_left, [np.int32(camera.light_point_left)], 255)
        cv2.fillPoly(region_mask_light_right, [np.int32(camera.light_point_right)], 255)

        region_mask_base = cv2.subtract(region_mask_base, region_mask_lightest)
        region_mask_base = cv2.subtract(region_mask_base, region_mask_light_left)
        region_mask_base = cv2.subtract(region_mask_base, region_mask_light_right)

        masked_lightest = cv2.bitwise_and(lightest_mask, region_mask_lightest)
        masked_light_left = cv2.bitwise_and(light_mask, region_mask_light_left)
        masked_light_right = cv2.bitwise_and(light_mask, region_mask_light_right)
        masked_base = cv2.bitwise_and(base_mask, region_mask_base)

        black_mask = cv2.bitwise_or(masked_lightest, masked_light_left)
        black_mask = cv2.bitwise_or(black_mask, masked_light_right)
        black_mask = cv2.bitwise_or(black_mask, masked_base)

        # --- Blue detection in top area ---
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        top_hsv = hsv_image[0:int(camera.LINE_HEIGHT/4), :]  # Only look at the top rows

        blue_mask_top = cv2.inRange(top_hsv, self.lower_blue, self.upper_blue)

        # Expand blue_mask_top to full image size (pad zeros below)
        full_blue_mask = np.zeros_like(black_mask)
        full_blue_mask[0:int(camera.LINE_HEIGHT/4), :] = blue_mask_top

        # Subtract blue from black mask
        black_mask = cv2.bitwise_and(black_mask, cv2.bitwise_not(full_blue_mask))
        # --- End blue detection section ---

        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) if not extra_erode else cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        black_mask = cv2.erode(black_mask, kernel, iterations=2)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  if not extra_erode else cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        black_mask = cv2.dilate(black_mask, kernel, iterations=1)

        contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_black_area]
        
        # TODO: black contour
        # all_points = np.concatenate(contours) if contours != [] else []
        # # print(all_points.shape)
        # # all_points = all_points.reshape(-1, 1, 2)
        # # print(all_points.shape)
        
        # self.black_contour = cv2.convexHull(all_points) if len(all_points) > 0 else None
        contours = [cnt for cnt in contours if any(p[0][1] > int(camera.LINE_HEIGHT/4) for p in cnt)]
        
        if len(contours) > 1:
            filtered_contours = []
            for contour in contours:
                if any(p[0][1] > camera.LINE_HEIGHT - int(camera.LINE_HEIGHT/20) and int(3*camera.LINE_HEIGHT/20) < p[0][0] < camera.LINE_WIDTH - int(3*camera.LINE_HEIGHT/20) for p in contour):
                    filtered_contours.append(contour)

            target_contours = filtered_contours if filtered_contours else contours

            close_contours = []
            for contour in target_contours:
                angle = self.calculate_angle(contour, validate=True)
                if abs(angle - self.last_angle) < 90:
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

        if self.black_contour is not None:
            contour_mask = np.zeros(self.gray_image.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [self.black_contour], -1, color=255, thickness=cv2.FILLED)
            self.black_mask = contour_mask

            if camera.X11:
                for c in contours:
                    if c is not self.black_contour:
                        cv2.drawContours(self.display_image, [c], -1, (255, 0, 0), int(camera.LINE_HEIGHT/100))
                    else:
                        cv2.drawContours(self.display_image, [c], -1, self.turn_color, int(camera.LINE_HEIGHT/100))

    # ========================================================================
    # RED
    # ========================================================================

    def find_red(self):
        if self.hsv_image is not None:
            mask_lower = cv2.inRange(self.hsv_image, (0, 230, 46), (10, 255, 255))
            mask_upper = cv2.inRange(self.hsv_image, (170, 230, 0), (179, 255, 255))
            mask = cv2.bitwise_or(mask_lower, mask_upper)

            if cv2.countNonZero(mask) > int(0.005 * camera.LINE_WIDTH * camera.LINE_HEIGHT):
                self.robot_state.count["red"] += 1
                return
        
        self.robot_state.count["red"] = 0
