import cv2
import numpy as np
import config
import laser_sensors
import camera
import motors
from tabulate import tabulate

def find() -> None:
    def align(tolerance: int, text: str, time_step: float = None) -> None:
        direction = 1
        while True:

            if time_step is None: 
                motors.run(config.evacuation_speed * direction * 0.4, -config.evacuation_speed * direction * 0.4)
            else:
                motors.run(config.evacuation_speed * direction * 0.4, -config.evacuation_speed * direction * 0.4, time_step)
                motors.run(0, 0, time_step)

            image = camera.capture_array()
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            mask = hsv_image.copy()

            if config.victim_count < 2:
                # Adjusting: H_max | H_min=35, H_max=75, S_min=0, S_max=255, V_min=0, V_max=255
                mask = cv2.inRange(hsv_image, (35, 128, 0), (75, 255, 255))
                mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
            else:
                # Adjusting: V_min | H_min=0, H_max=10, S_min=230, S_max=255, V_min=46, V_max=255
                # Adjusting: S_min | H_min=170, H_max=179, S_min=230, S_max=255, V_min=0, V_max=255
                mask_lower = cv2.inRange(hsv_image, (0, 230, 46), (10, 255, 255))
                mask_upper = cv2.inRange(hsv_image, (170, 230, 0), (179, 255, 255))
                mask = cv2.bitwise_or(mask_lower, mask_upper)
                mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
                
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                if config.X11: cv2.imshow("image", image)
                continue

            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) < 2000:
                if config.X11: cv2.imshow("image", image)
                continue

            x, y, w, h = cv2.boundingRect(largest_contour)

            direction = -1 if config.EVACUATION_WIDTH/2 - int(x+ w/2) >= 0 else 1

            if config.X11:
                cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
                cv2.circle(image, (int(x + w / 2), int(y + h / 2)), 3, (255, 0, 0), 1)
                cv2.imshow("image", image)

            if config.EVACUATION_WIDTH/2 - (x + w / 2) < tolerance and config.EVACUATION_WIDTH/2 - (x + w / 2) > -tolerance: break

            print(config.update_log([f"({text})", "green" if config.victim_count < 2 else "red", f"{config.EVACUATION_WIDTH/2 - (x + w / 2)}"], [24, 10, 10]))

    config.update_log(["TRIANGLE", "initial alignment"], [24, 24])
    print()
    align(tolerance=10, text="Initial Alignment")
    motors.run(0, 0)

    motors.run(0, 0, 0.3)
    motors.run_until(config.evacuation_speed, config.evacuation_speed, laser_sensors.read, 1, "<=", 35)
    motors.run(0, 0, 0.3)
    motors.run_until(-config.evacuation_speed, -config.evacuation_speed, laser_sensors.read, 1, ">=", 35)
    motors.run(0, 0)

    print("(TRIANGLE SEARCH) Fine alignment")
    align(tolerance=3, text="Fine Alignment", time_step=0.1)

    motors.run(0, 0)