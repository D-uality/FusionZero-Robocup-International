import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))

if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import time
from typing import Optional
import operator

from motors import cMOTORS
from camera import cCAMERA
from utils import debug
import led
import cv2
import numpy as np

class cRobotState():
    def __init__(self):
        self.main_loop_count = 0

class cLine():
    def __init__(self, camera, motors):
        self.straight_speed = 30
        self.turn_multi = 1.5
        self.min_black_area = 1000
        self.min_green_area = 1000
        self.base_black = 50
        self.light_black = 60
        self.lightest_black = 100

        self.camera = camera
        self.motors = motors

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
    
    def follow_line(self) -> None:
        start_time = time.time()

        t0 = time.time()
        self.image = self.camera.capture_array()
        t1 = time.time()
        self.display_image = self.image.copy()

        self.find_black()
        t2 = time.time()

        self.find_green()
        t3 = time.time()

        self.green_check()
        t4 = time.time()

        self.calculate_angle(self.black_contour)
        t5 = time.time()

        if self.green_signal == "Double":
            self.angle = 90
            error = 0
            self.motors.run(30, -30, 2.3)
        else:
            error = int(self.turn_multi * (self.angle - 90))
            if abs(error) < 10:
                error = 0
            self.motors.run(self.straight_speed + error, self.straight_speed - error)

        
        if self.display_image is not None and self.camera.X11:
            small_image = cv2.resize(self.display_image, (0, 0), fx=0.5, fy=0.5)

            cv2.imshow("line", np.uint8(small_image))
            cv2.waitKey(1)

        t6 = time.time()

        elapsed_time = time.time() - start_time
        if self.green_signal == "Double":
            elapsed_time = elapsed_time - 1.5
        fps = int(1.0 / elapsed_time) if elapsed_time > 0 else 0

        # print(f"[TIMING] capture={t1-t0:.3f}s black={t2-t1:.3f}s green={t3-t2:.3f}s check={t4-t3:.3f}s angle={t5-t4:.3f}s display={t6-t5:.3f}s total={elapsed_time:.3f}s")

        return fps, error, self.green_signal

    

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
        if 0 <= x_check < self.camera.LINE_WIDTH and 0 <= y_check < self.camera.LINE_HEIGHT and self.black_mask is not None:
            if self.black_check((x_check, y_check)):
                return box

        return None

    def black_check(self, check_point):
        if 0 <= check_point[0] < self.camera.LINE_WIDTH and 0 <= check_point[1] < self.camera.LINE_HEIGHT and self.black_mask is not None:
            # Create a 20x20 region around the point
            y_start = max(0, check_point[1] - 10)
            y_end = min(self.camera.LINE_HEIGHT, check_point[1] + 10)
            x_start = max(0, check_point[0] - 10)
            x_end = min(self.camera.LINE_WIDTH, check_point[0] + 10)
            region = self.black_mask[y_start:y_end, x_start:x_end]

            # If display image, draw the check point
            if self.display_image is not None and self.camera.X11:
                cv2.circle(self.display_image, check_point, 20, (0, 255, 255), -1)

            # Check if any pixel in the region is black
            return np.any(region == 255)
        return False

    def green_hold(self):
        if self.green_signal != None and self.prev_green_signal != self.green_signal:
            self.last_seen_green = 0
        else:
            self.last_seen_green += 1

        if self.last_seen_green < 20:
            if self.green_signal != "Double":
                print("Holding green")
                self.prev_green_signal = self.green_signal

    def find_green(self):
        self.green_contours = []
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_image, (40, 50, 50), (90, 255, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        green_mask = cv2.erode(green_mask, kernel, iterations=2)
        contours, _ = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        green_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_green_area]

        for contour in green_contours:
            if all(p[0][1] > int(self.camera.LINE_HEIGHT/3.5) for p in contour) and contour is not None and len(contour) > 0:
                self.green_contours.append(contour)

        if self.green_contours and self.display_image is not None and self.camera.X11:
            cv2.drawContours(self.display_image, self.green_contours, -1, (0, 0, 255), 2)


    # BLACK
    def calculate_angle(self, contour=None, validate=False):
        ref_point = None
        if contour is not None:
            ref_point = self.calculate_top_contour(contour, validate)

            left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= 10]
            right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= self.camera.LINE_WIDTH - 10]

            if self.green_signal == "Left":     
                point = min(contour, key=lambda p: p[0][0], default=None)
                ref_point = tuple(point[0])
            elif self.green_signal == "Right":  
                point = max(contour, key=lambda p: p[0][0], default=None)
                ref_point = tuple(point[0])
            elif ref_point[1] > 50 and (left_edge_points or right_edge_points):
                y_avg_left = int(np.mean([p[1] for p in left_edge_points])) if left_edge_points else None
                y_avg_right = int(np.mean([p[1] for p in right_edge_points])) if right_edge_points else None

                if y_avg_left is not None and (self.last_angle < 90 or y_avg_right is None or y_avg_left < y_avg_right):
                    ref_point = (0, y_avg_left)
                elif y_avg_right is not None:
                    ref_point = (self.camera.LINE_WIDTH - 1, y_avg_right)
                
        if ref_point is not None:
            bottom_center = (self.camera.LINE_WIDTH // 2, self.camera.LINE_HEIGHT)
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

            if self.display_image is not None and self.camera.X11:
                cv2.line(self.display_image, ref_point, bottom_center, (0, 0, 255), 2)
        
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
        if self.display_image is not None and validate is False and self.camera.X11:
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

        # Define regions using self.camera.LINE_WIDTH and LINE_HEIGHT
        light_points = np.array([
            (self.camera.LINE_WIDTH - 55, 0),
            (60, 0),
            (60, int(self.camera.LINE_HEIGHT / 2.5) - 1),
            (self.camera.LINE_WIDTH - 55, int(self.camera.LINE_HEIGHT / 2.35) - 1)
        ], dtype=np.int32)

        lightest_points = np.array([
            (self.camera.LINE_WIDTH - 55, int(self.camera.LINE_HEIGHT / 2.35)),
            (60, int(self.camera.LINE_HEIGHT / 2.5)),
            (40, self.camera.LINE_HEIGHT - 30),
            (self.camera.LINE_WIDTH - 50, self.camera.LINE_HEIGHT - 25)
        ], dtype=np.int32)

        # Create region masks
        region_mask_lightest = np.zeros_like(gray, dtype=np.uint8)
        region_mask_light = np.zeros_like(gray, dtype=np.uint8)
        region_mask_base = np.ones_like(gray, dtype=np.uint8) * 255

        cv2.fillPoly(region_mask_lightest, [lightest_points], 255)
        cv2.fillPoly(region_mask_light, [light_points], 255)

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
                    if highest_point[0][1] > self.camera.LINE_HEIGHT / 2:
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

            if self.camera.X11:
                for c in contours:
                    color = (255, 255, 0) if c is self.black_contour else (255, 0, 0)
                    cv2.drawContours(self.display_image, [c], -1, color, 2)
                
                    # for pt in c:
                    #     x, y = pt[0]
                    #     # a filled circle of radius 1 (you can bump that up if you like)
                    #     cv2.circle(self.display_image, (x, y), radius=1, color=color, thickness=-1)

        # if self.camera.X11:
        #     cv2.imshow("black_mask", black_mask)
        #     cv2.waitKey(1)

robot_state = cRobotState()

def main(line_follow) -> None:
    global robot_state

    something_besides_line_follow = False
    if something_besides_line_follow:
        pass
    else:
        led.on()
        fps, error, green_signal = line_follow.follow_line()

        debug(["LINE", str(robot_state.main_loop_count), str(fps), str(error), str(green_signal)], [10, 15, 10, 10, 15])

    # Update the loop count
    robot_state.main_loop_count += 1