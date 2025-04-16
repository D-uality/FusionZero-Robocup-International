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
        self.straight_speed = 25
        self.turn_multi = 1.6
        self.min_black_area = 1000
        self.min_green_area = 1000

        self.camera = camera
        self.motors = motors

        self.last_angle = 90
        self.angle = 90
        self.image = None
        self.display_image = None
        self.green_contours = None
        self.green_signal = None
        self.black_contour = None
        self.black_mask = None
    
    def follow_line(self) -> None:
        start_time = time.time()
        self.image = self.camera.capture_array()
        self.display_image = self.image.copy()
        self.find_black()
        self.find_green()
        self.green_check()
        self.calculate_angle(self.black_contour)

        if self.green_signal == "Double":
            self.angle = 90
            error = 0
            # self.motors.run(30, -30, 1.5)
            self.motors.pause()
        else:
            error = int(self.turn_multi * (self.angle - 90))
            self.motors.run(self.straight_speed + error, self.straight_speed - error)
        
        if self.display_image is not None and self.camera.X11:
            cv2.imshow("line", np.uint8(self.display_image))
            
            # if self.black_mask is not None:
            #     cv2.imshow("black", np.uint8(self.black_mask))
            cv2.waitKey(1)
        
        elapsed_time = time.time() - start_time
        if self.green_signal == "Double":
            elapsed_time = elapsed_time - 1.5
        fps = int(1.0 / elapsed_time) if elapsed_time > 0 else 0

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
                    return

                # Take 2 Rightmost points
                right_points = sorted(valid_rect, key=lambda p: p[0])[-2:]
                # Average right points
                right_x = int(sum(p[0] for p in right_points) / len(right_points))
                right_y = int(sum(p[1] for p in right_points) / len(right_points))

                # Check black on right
                check_point = (right_x + 20, right_y)
                if self.black_check(check_point):
                    self.green_signal = "Left"
                    return

                self.green_signal = "Left"

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

        green_valid = False

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

    def find_green(self):
        self.green_contours = None
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_image, (40, 50, 50), (90, 255, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        green_mask = cv2.erode(green_mask, kernel, iterations=2)
        contours, _ = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        self.green_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_green_area]

        if self.green_contours is not None and self.display_image is not None and self.camera.X11:
            cv2.drawContours(self.display_image, self.green_contours, -1, (0, 0, 255), 2)


    # BLACK
    def calculate_angle(self, contour=None, validate=False):
        ref_point = None
        if contour is not None:
            ref_point = self.calculate_top_contour(contour)

            left_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] <= 10]
            right_edge_points = [(p[0][0], p[0][1]) for p in contour if p[0][0] >= self.camera.LINE_WIDTH - 10]

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
        
    def calculate_top_contour(self, contour):
        # Take highest 2 points
        top_points = sorted(contour, key=lambda p: p[0][1])
        top_points = top_points[:2]

        if not top_points:
            return None

        if self.display_image is not None and self.camera.X11:
            for point in top_points:
                cv2.circle(self.display_image, tuple(point[0]), 3, (0, 0, 255), -1)

        # Calculate top point
        x_avg = int(sum(p[0][0] for p in top_points) / len(top_points))
        y_avg = int(sum(p[0][1] for p in top_points) / len(top_points))


        return int(x_avg), int(y_avg)

    def find_black(self):
        self.black_contour = None
        self.black_mask = None
        image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        black_mask = cv2.inRange(image, 0, 30)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 
        black_mask = cv2.erode(black_mask, kernel, iterations=3)
        black_mask = cv2.dilate(black_mask, kernel, iterations=10)
        contours, _ = cv2.findContours(black_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_black_area]

        if len(contours) > 1:
            # Contours within 15 degrees from last angle
            close_contours = []
            for contour in contours:
                angle = self.calculate_angle(contour, validate=True)
                if abs(angle - self.last_angle) < 15:
                    close_contours.append(contour)

            if len(close_contours) == 0:
                # pick the contour which is closest to the previous angle and set as self.black_contour
                closest_contour = min(contours, key=lambda c: abs(self.calculate_angle(c, validate=True) - self.last_angle))
                self.black_contour = closest_contour

            # Highest contour
            tallest_contours = []
            if len(close_contours) > 1:
                for contour in close_contours:
                    highest_point = max(contour, key=lambda p: p[0][1])
                    if highest_point[0][1] > self.camera.LINE_HEIGHT / 2:
                        tallest_contours.append(contour)
            elif len(close_contours) == 1:
                self.black_contour = close_contours[0]

            # Biggest area
            if len(tallest_contours) > 1:
                self.black_contour = max(tallest_contours, key=lambda c: cv2.contourArea(c))
            elif len(tallest_contours) == 1:
                self.black_contour = tallest_contours[0]

        elif len(contours) == 1:
            self.black_contour = contours[0]
        if self.black_contour is not None:
            contour_mask = np.zeros(image.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [self.black_contour], -1, color=255, thickness=cv2.FILLED)
            self.black_mask = contour_mask

            if self.camera.X11:
                # Draw all contours in blue, and the largest in cyan
                for c in contours:
                    color = (255, 255, 0) if c is self.black_contour else (255, 0, 0)
                    cv2.drawContours(self.display_image, [c], -1, color, 2)

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