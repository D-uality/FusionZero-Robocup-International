from core.shared_imports import os, sys, time, randint, Optional, operator, cv2, np
from core.utilities import *
from hardware.robot import *
import behaviours.evacuation_zone as evacuation_zone

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
        self.speed = 30
        self.turn_multi = 1.5

        self.min_black_area = 1000
        self.min_green_area = 3000
        self.base_black = 50
        self.light_black = 80
        self.lightest_black = 100
        self.silver = 253

        self.lower_blue = np.array([100, 100, 100])
        self.upper_blue = np.array([130, 255, 255])
        
        self.turn_color = (0, 255, 0)
        self.prev_side = None
  
        self.last_angle = 90
        self.angle = 90
        self.turn = 0
        self.low_turn = False
        self.last_check_front = 0
        self.image = None
        self.hsv_image = None
        self.gray_image = None
        self.display_image = None
        self.blue_contours = None
        self.green_contours = None
        self.green_signal = None
        self.prev_green_signal = None
        self.last_seen_green = 100
        self.black_contour = None
        self.black_mask = None
        self.__timing = False
    
    def follow(self, starting=False) -> None:
        overall_start = time.perf_counter()
        timings = {}

        t0 = time.perf_counter()
        self.image = camera.capture_array()
        self.display_image = self.image.copy()
        timings['capture'] = time.perf_counter() - t0

        t0 = time.perf_counter()
        self.find_black()
        timings['find_black'] = time.perf_counter() - t0

        if self.black_contour is not None:
            t0 = time.perf_counter()
            self.find_green()
            timings['find_green'] = time.perf_counter() - t0

            t0 = time.perf_counter()
            self.green_check()
            timings['green_check'] = time.perf_counter() - t0

            t0 = time.perf_counter()
            self.calculate_angle(self.black_contour)
            timings['calculate_angle'] = time.perf_counter() - t0

            if not starting:
                t0 = time.perf_counter()
                self.__turn()
                timings['__turn'] = time.perf_counter() - t0
        elif not starting and robot_state.trigger["uphill"] is False and robot_state.trigger["downhill"] is False:
            self.gap_handling()

        if self.display_image is not None and camera.X11:
            t0 = time.perf_counter()
            show(np.uint8(self.display_image), display=camera.X11, name="line")
            timings['display'] = time.perf_counter() - t0

        total_elapsed = time.perf_counter() - overall_start
        fps = int(1.0 / total_elapsed) if total_elapsed > 0 else 0

        # Format timing info in one line
        # timing_line = f"[Timing] Total: {total_elapsed:.4f}s | FPS: {fps} | " + " | ".join(
        #     f"{key}: {value:.4f}s" for key, value in timings.items()
        # )
        # print(timing_line)

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
            for i in range(5):
                motors.run(-30+i*6, -30+i*6, 0.20)
        elif self.green_signal == "Double":
            v1, v2, t = 35, -35, 3.8
            f = b = f_after = b_after = 0

            if robot_state.trigger["tilt_left"]:
                v1, v2, t = 40, -15, 3
                f = 0.5 if robot_state.trigger["uphill"] else 0
            elif robot_state.trigger["tilt_right"]:
                v1, v2, t = -15, 40, 3
                f = 0.5 if robot_state.trigger["uphill"] else 0
            elif robot_state.trigger["uphill"]:
                f, t = 1.3, 3
                v1 = v1
                v2 = v2 * 1.5
            if robot_state.trigger["downhill"]:
                v1 -= 5
                v2 += 5
                t -= 0.3
                b = 1

            motors.run(-30, -30, b)
            motors.run(30, 30, f)
            motors.run(v1, v2, t)
            motors.run(0, 0, 0.5)
            self.run_till_camera(v1, v2, 15)
        else:
            v1 = self.speed + self.turn
            v2 = self.speed - self.turn

            # if time.perf_counter() - self.last_check_front > 0.5 and (robot_state.trigger["downhill"] or robot_state.trigger["uphill"]) and robot_state.trigger["tilt_left"] is False and robot_state.trigger["tilt_right"] is False:
            #     self.low_turn = self.check_front()
            #     self.last_check_front = time.perf_counter()

            if robot_state.trigger["downhill"]:
                speed = 18
                v1 = speed + self.turn
                v2 = speed - self.turn
                v1 = min(v1, max(-v2//5, speed))
                v2 = min(v2, max(-(speed + self.turn)//5, speed))
                if self.green_signal == None and self.low_turn and robot_state.trigger["tilt_left"] is False and robot_state.trigger["tilt_right"] is False:
                    v1 = speed + 0.01 * self.turn
                    v2 = speed - 0.01 * self.turn

            elif robot_state.trigger["uphill"]:
                v1 += 5
                v2 += 5
                if v1 < 0: v1 = max(-30, v1)
                if v2 < 0: v2 = max(-30, v2)
                if self.green_signal == None and self.low_turn and robot_state.trigger["tilt_left"] is False and robot_state.trigger["tilt_right"] is False:
                    v1 = 35 + 0.03 * self.turn
                    v2 = 35 - 0.03 * self.turn
            
            if robot_state.trigger["tilt_left"] and v2 < 0: v2 = v2 // 2

            if robot_state.trigger["tilt_right"] and v1 < 0: v1 = v1 // 2

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
                show(np.uint8(self.display_image), camera.X11, name="line")
            
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
        # if y_avg < int(camera.LINE_HEIGHT / 3) and robot_state.trigger["downhill"] is False:
        if y_avg < int(camera.LINE_HEIGHT / 3):
            self.green_signal = "Approach"
            return None
        elif y_avg < int(camera.LINE_HEIGHT / 2) and robot_state.trigger["uphill"] is True:
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
    def gap_handling(self):
        print("Gap Detected!")
        motors.run(0, 0, 1)
        motors.run(-20, -20)

        self.__wait_for_black_contour()
        motors.run(-20, -20, 0.2)
        motors.run(0, 0, 0.3)

        self.__align_to_contour_angle()
        motors.run(0, 0, 0.2)

        f = 2.5 + 0.5 * (robot_state.last_uphill < 100)
        if not self.__move_and_check_black(f):
            print("No black found, retrying...")

            motors.run(-25, -25, f+0.2)
            motors.run(0, 0, 0.2)

            self.__align_to_contour_angle()
            motors.run(0, 0, 0.2)

            if not self.__move_and_check_black(f+0.4):
                print("Still no black. Exiting gap handler.")
                motors.run(-25, -25, f+0.6)

        print("Gap handling complete.")
    
    def __wait_for_black_contour(self):
        while True:
            self.image = camera.capture_array()
            self.display_image = self.image.copy()
            self.find_black()

            if self.black_contour is not None:
                if any(p[0][1] >= camera.LINE_HEIGHT - 5 for p in self.black_contour) and cv2.contourArea(self.black_contour) > 5000:
                    print("Black contour found.")
                    break

            if self.display_image is not None and camera.X11:
                show(np.uint8(self.display_image), display=camera.X11, name="line")

    def __align_to_contour_angle(self):
        while True:
            self.image = camera.capture_array()
            self.display_image = self.image.copy()
            self.find_black()

            if self.black_contour is not None and self.black_mask is not None:
                mask = self.black_mask.copy()

                # remove everything above LINE_HEIGHT - 80 if last uphill was recent
                if robot_state.last_uphill < 100:
                    mask[0:camera.LINE_HEIGHT - 120, :] = 0

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
                if abs(angle - 90) < 3:
                    break
                elif angle > 90:
                    motors.run(15, -15)
                else:
                    motors.run(-15, 15)

                # --- DEBUG DRAW ---
                if self.display_image is not None and camera.X11:
                    # Draw poly approximation
                    cv2.polylines(self.display_image, [approx], isClosed=True, color=(255, 0, 255), thickness=5)

                    # Draw angle line
                    cv2.line(self.display_image, bottom_point, top_point, (0, 255, 0), 2)
                    cv2.circle(self.display_image, top_point, 5, (0, 255, 255), 2)
                    cv2.circle(self.display_image, bottom_point, 5, (255, 255, 0), 2)

                    show(np.uint8(self.display_image), camera.X11, name="line")

    def __move_and_check_black(self, duration: float) -> bool:
        motors.run(25, 25, duration)
        motors.run(0, 0, 0.3)

        self.image = camera.capture_array()
        self.display_image = self.image.copy()
        self.find_black()

        if self.black_contour is not None:
            return True
        return False

    def calculate_angle(self, contour=None, validate=False):
        timing = {}
        start = time.perf_counter()

        if not validate:
            self.angle = 90
            if contour is None:
                return 90

        t0 = time.perf_counter()
        ref_point = self.calculate_top_contour(contour, validate) if contour is not None else None
        timing['top_contour'] = time.perf_counter() - t0

        t0 = time.perf_counter()
        if contour is not None and self.green_signal in ["Left", "Right"]:
            self.prev_side = self.green_signal
            point = min(contour, key=lambda p: p[0][0]) if self.green_signal == "Left" else max(contour, key=lambda p: p[0][0])
            ref_point = tuple(point[0])
            timing['green_case'] = time.perf_counter() - t0
            t1 = time.perf_counter()
            return_val = self._finalize_angle(ref_point, validate)
            timing['finalize'] = time.perf_counter() - t1
            # print("[Angle Timing]", " | ".join(f"{k}: {v:.4f}s" for k, v in timing.items()))
            return return_val
        timing['green_case'] = time.perf_counter() - t0

        t0 = time.perf_counter()
        if contour is not None and ref_point is not None:
            left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= 20]
            right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= camera.LINE_WIDTH - 20]

            if ref_point[1] < (50 + robot_state.trigger["uphill"] * (camera.LINE_HEIGHT // 2 - 70) + 20 * (robot_state.last_downhill < 100)):
                # if robot_state.last_downhill > 100: self.prev_side = None
                timing['edges'] = time.perf_counter() - t0
                t1 = time.perf_counter()
                return_val = self._finalize_angle(ref_point, validate)
                timing['finalize'] = time.perf_counter() - t1
                # print("[Angle Timing]", " | ".join(f"{k}: {v:.4f}s" for k, v in timing.items()))
                return return_val

            if self.green_signal != "Approach" and (left_edge_points or right_edge_points):
                y_avg_left = int(np.mean([p[1] for p in left_edge_points])) if left_edge_points else None
                y_avg_right = int(np.mean([p[1] for p in right_edge_points])) if right_edge_points else None

                if self.prev_side is None:
                    # self.prev_side = "Left" if self.angle < 90 else "Right" if self.angle > 90 else None
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
        timing['edges'] = time.perf_counter() - t0

        t0 = time.perf_counter()
        return_val = self._finalize_angle(ref_point, validate) if ref_point is not None else 90
        timing['finalize'] = time.perf_counter() - t0

        # print("[Angle Timing]", " | ".join(f"{k}: {v:.4f}s" for k, v in timing.items()))
        # print(self.prev_side)
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

        trigger_states = ", ".join([k for k, v in robot_state.trigger.items() if v])
        info_text = f"Ref: {ref_point}, Angle: {angle}, Triggers: {trigger_states}"

        # Put text at the top-left corner
        cv2.putText(
            self.display_image,
            info_text,
            (10, 25),  # (x, y) coordinates
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3,  # Font scale
            self.turn_color,  # Text color (white)
            1,  # Thickness
            cv2.LINE_AA
        )
    
    def calculate_top_contour(self, contour, validate):
        if contour is None:
            return camera.LINE_WIDTH // 2, 0

        # Create mask from contour
        contour_mask = np.zeros_like(self.gray_image, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

        # Define scan band
        min_y = 40 if self.green_signal == "Approach" else min(pt[0][1] for pt in contour)
        if robot_state.trigger["uphill"]:
            min_y = camera.LINE_HEIGHT // 2 - 40
        elif robot_state.trigger["downhill"]:
            min_y = 50  
        max_y = min(camera.LINE_HEIGHT, min_y + 10)

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

        # # Draw sampled points
        # if self.display_image is not None and camera.X11:
        #     for pt in points:
        #         cv2.circle(self.display_image, pt, 3, (255, 0, 0), -1)  # blue dot

        # average for stability
        x_avg = int(np.mean([pt[0] for pt in points]))
        y_avg = int(np.mean([pt[1] for pt in points]))
        return x_avg, y_avg
    
    def check_front(self):
        image = evac_camera.capture(False)
        display_image = image.copy()

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        black_mask = cv2.inRange(gray_image, 0, self.light_black)

        if display_image is not None and camera.X11:
            show(np.uint8(black_mask), camera.X11, name="front")
        
        return cv2.countNonZero(black_mask) > 0  

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

        region_mask_base = cv2.subtract(region_mask_base, region_mask_lightest)
        region_mask_base = cv2.subtract(region_mask_base, region_mask_light)

        masked_lightest = cv2.bitwise_and(lightest_mask, region_mask_lightest)
        masked_light = cv2.bitwise_and(light_mask, region_mask_light)
        masked_base = cv2.bitwise_and(base_mask, region_mask_base)

        black_mask = cv2.bitwise_or(masked_lightest, masked_light)
        black_mask = cv2.bitwise_or(black_mask, masked_base)

        # --- Blue detection in top area ---
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        top_hsv = hsv_image[0:50, :]  # Only look at the top 50 rows

        blue_mask_top = cv2.inRange(top_hsv, self.lower_blue, self.upper_blue)

        # Expand blue_mask_top to full image size (pad zeros below)
        full_blue_mask = np.zeros_like(black_mask)
        full_blue_mask[0:50, :] = blue_mask_top

        # Subtract blue from black mask
        black_mask = cv2.bitwise_and(black_mask, cv2.bitwise_not(full_blue_mask))
        # --- End blue detection section ---

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 
        black_mask = cv2.erode(black_mask, kernel, iterations=2)
        black_mask = cv2.dilate(black_mask, kernel, iterations=5)

        contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_black_area]
        contours = [cnt for cnt in contours if any(p[0][1] > 50 for p in cnt)]
        # contours = [cnt for cnt in contours if (cv2.contourArea(cnt) > self.min_black_area and any(p[0][1] > 50 for p in cnt) and any(p[0][1] < 50 for p in cnt))]

        if len(contours) > 1:
            filtered_contours = []
            for contour in contours:
                if any(p[0][1] > camera.LINE_HEIGHT - 10 and 30 < p[0][0] < camera.LINE_WIDTH - 30 for p in contour):
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
                        cv2.drawContours(self.display_image, [c], -1, (255, 0, 0), 2)
                    else:
                        cv2.drawContours(self.display_image, [c], -1, self.turn_color, 2)
    
    # SILVER
    def find_silver(self):
        if self.hsv_image is None:
            robot_state.count["silver"] = 0
            return 

        # Create blue mask
        mask = cv2.inRange(self.hsv_image, self.lower_blue, self.upper_blue)

        # Find contours in the blue mask
        self.blue_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Silver detection logic
        blue_pixels = cv2.countNonZero(mask)
        total_pixels = self.image.shape[0] * self.image.shape[1]
        print(f"[DEBUG] Blue pixel ratio: {blue_pixels / total_pixels:.3f} ({blue_pixels} / {total_pixels})")
        if blue_pixels / total_pixels >= 0.1: robot_state.count["silver"] += 1
        elif blue_pixels / total_pixels >= 0.03 and robot_state.trigger["downhill"]: robot_state.count["silver"] += 2
        elif robot_state.count["silver"] > 0: robot_state.count["silver"] -= 1
        print(robot_state.count["silver"])

    
    # RED
    def find_red(self):
        if self.hsv_image is not None:
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
    led.on()
    while time.perf_counter() - start_time < 3:
        line_follow.follow(True)
    fps = error = 0
    green_signal = None

    colour_values = colour_sensors.read()
    touch_values = touch_sensors.read()
    gyro_values = gyroscope.read()
    touch_check(robot_state, touch_values)
    ramp_check(robot_state, gyro_values)
    update_triggers(robot_state)

    line_follow.find_silver()
    line_follow.find_red()

    if robot_state.count["silver"] >= 5:
        print("Silver Found!")
        robot_state.count["silver"] = 0
        motors.pause()
        evacuation_zone.main()
        robot_state.trigger["evacuation_zone"] = True
    elif robot_state.count["red"] >= 5:
        print("Red Found!")
        motors.run(0, 0, 8)
        robot_state.count["red"] = 0
    elif robot_state.count["touch"] > 3:
        robot_state.count["touch"] = 0
        avoid_obstacle(line_follow, robot_state)
    else:
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

    if robot_state.count["uphill"] > 5:
        robot_state.trigger["uphill"] = True
    else:
        robot_state.trigger["uphill"] = False
         
    if robot_state.count["downhill"] > 5:
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
    
    if robot_state.last_uphill <= 20 and robot_state.count["downhill"] > 0 and robot_state.count["tilt_right"] < 1 and robot_state.count["tilt_left"] < 1:
        robot_state.trigger["downhill"] = False
        robot_state.trigger["seasaw"] = True
        robot_state.last_uphill = 100
    else:
        robot_state.trigger["seasaw"] = False

    if robot_state.trigger != prev_triggers:
        if robot_state.main_loop_count <= 10:
            robot_state.triggers = prev_triggers

def avoid_obstacle(line_follow: LineFollower, robot_state: RobotState) -> None:
    if robot_state.trigger["downhill"] or robot_state.trigger["uphill"] or robot_state.trigger["tilt_left"] or robot_state.trigger["tilt_right"]:
        touch_count = 0
        while True:
            touch_values = touch_sensors.read()
            if sum(touch_values) == 0:
                touch_count += 1
            elif sum(touch_values) == 2:
                touch_count = 0
                motors.run(25, 25)
            elif touch_values[0] == 0:
                motors.run(-15, 25)
                touch_count = 0
            elif touch_values[1] == 0:
                motors.run(25, -15)
                touch_count = 0
            if touch_count > 5:
                break

    motors.run(0, 0)

    for i in range(100): 
        right_value = laser_sensors.read([2])[0]
        left_value = laser_sensors.read([0])[0]
        time.sleep(0.001)

    # Clockwise if left > right
    side_values = [left_value, right_value]

    # Immediately update OLED for obstacle detection
    # oled_display.reset()
    # oled_display.text("Obstacle Detected", 0, 0, size=10)

    debug(["OBSTACLE", "FINDING SIDE", ", ".join(list(map(str, side_values)))], [24, 50, 14])
    
    # Clockwise if left > right, else random
    if side_values[0] <= 40 or side_values[1] <= 40:
        direction = "cw" if side_values[0] > side_values[1] else "ccw"
    else:
        direction = "cw" if randint(0, 1) == 0 else "ccw"

    if direction == "cw":
        v1 = -30
        v2 =  30 * 0.8
        laser_pin = 2
        if robot_state.count["downhill"] > 0:
            v1 = -30 * 0.7
    else:
        v1 =  30 * 0.8
        v2 = -30
        laser_pin = 0
        if robot_state.count["downhill"] > 0:
            v2 = -30 * 0.7

    # SETUP
    # Over turn passed obstacle
    # oled_display.text("Turning till obstacle", 0, 10, size=10

    if robot_state.count["downhill"] > 0:
        motors.run(-30-10, -30-10, 1)
        print("Backing up")
    elif robot_state.count["uphill"] > 5:
        motors.run(-30, -30, 0.2)
    else:
        motors.run(-30, -30, 0.4)

    for i in range(3):
        if robot_state.count["downhill"] > 0:
            motors.run_until(v1, v2, laser_sensors.read, 1, ">=", 20, "TURNING PAST OBSTACLE (MIDDLE)")
        else:
            motors.run_until(v1, v2, laser_sensors.read, 1, ">=", 15, "TURNING PAST OBSTACLE (MIDDLE)")
        motors.run(v1, v2, 0.01)

    if robot_state.count["uphill"] > 5:
        motors.run(v1, v2, 0.2)
    else:
        motors.run(v1, v2, 0.7)

    # Circle obstacle
    v1 =  30 if direction == "cw" else -30
    v2 = -30 if direction == "cw" else 30
    laser_pin = 2 if direction == "cw" else 0
    colour_pin = 2

    initial_sequence = True

    circle_obstacle(30, 30, laser_pin, colour_pin, "<=", 10, "FORWARDS TILL OBSTACLE", initial_sequence, direction)

    while True:
        if circle_obstacle(30, 30, laser_pin, colour_pin, ">=", 18, "FORWARDS TILL NOT OBSTACLE", initial_sequence, direction): pass
        elif not initial_sequence: break
        if robot_state.count["uphill"] > 1:
            motors.run(30, 30, 0.1)
        # if robot_state.count["uphill"] < 5 and robot_state.last_downhill > 100:
        #     motors.run(30, 30, 0.4)
        elif robot_state.last_downhill < 100:
            motors.run(-30, -30, 0.3)
        motors.run(0, 0, 0.15)

        if circle_obstacle(v1, v2, laser_pin, colour_pin, "<=", 10, "TURNING TILL OBSTACLE", initial_sequence, direction): pass
        elif not initial_sequence: break
        motors.run(0, 0, 0.15)

        if circle_obstacle(v1, v2, laser_pin, colour_pin, ">=", 20, "TURNING TILL NOT OBSTACLE", initial_sequence, direction): pass
        elif not initial_sequence: break
        motors.run(v1, v2, 0.4)
        motors.run(0, 0, 0.15)

        if circle_obstacle(30, 30, laser_pin, colour_pin, "<=", 13, "FORWARDS TILL OBSTACLE", initial_sequence, direction): pass
        elif not initial_sequence: break
        motors.run(0, 0, 0.15)

        initial_sequence = False

    # oled_display.reset()
    # oled_display.text("Black Found", 0, 0, size=10)
    debug(["OBSTACLE", "FOUND BLACK"], [24, 50])
    print("hi")
    if robot_state.count["downhill"] < 5: motors.run(30, 30, 0.2)
    else: 
        print("Backing Up")
        motors.run(-30, -30, 0.5)
    
    if robot_state.count["uphill"] > 0: loops = 1
    else: loops = 3

    if robot_state.count["downhill"] > 0:
        motors.run(-v1, -v2, 2.3)
        motors.run(-30, -30, 0.5)
        motors.run(0, 0, 2)

    for i in range(loops):
        if direction == "cw":
            line_follow.run_till_camera(-v1, -v2-5, 20)
        else:
            line_follow.run_till_camera(-v1-5, -v2, 20)
    
def circle_obstacle(v1: float, v2: float, laser_pin: int, colour_pin: int, comparison: str, target_distance: float, text: str = "", initial_sequence: bool = False, direction: str = "") -> bool:
    global camera
    if   comparison == "<=": comparison_function = operator.le
    elif comparison == ">=": comparison_function = operator.ge

    while True:
        show(np.uint8(camera.capture_array()), camera.X11, name="line") 
        laser_value = laser_sensors.read([laser_pin])[0]
        colour_values = colour_sensors.read()
        touch_values = touch_sensors.read()

        debug(["OBSTACLE", text, f"{laser_value}"], [24, 50, 10])

        motors.run(v1, v2)
        if laser_value is not None:
            if comparison_function(laser_value, target_distance) and laser_value != 0: return True

        if colour_values[colour_pin] <= 30 and initial_sequence == False:
            return False
        
        if sum(touch_values) < 2:
            if v1 < 0 or v2 < 0:
                if direction == "cw": motors.run(v1, -v2)
                else: motors.run(-v1, v2)
            else:
                if direction == "cw": motors.run(-v1, v2)
                else: motors.run(v1, -v2)
"""
TESTING
"""

if __name__ == "__main__": pass