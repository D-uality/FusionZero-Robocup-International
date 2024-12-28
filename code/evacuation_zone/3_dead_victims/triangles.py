import cv2
import numpy as np
import logging
import config
import laser_sensors
import camera
import motors

def find() -> None:
    def align(tolerance: int, text: str, time_step: float = None) -> None:
        direction = 1
        while True:
            print(f"({text})    |    ", end="    ")

            if time_step is None: motors.run(config.evacuation_speed * direction * 0.4, -config.evacuation_speed * direction * 0.4)
            else:
                motors.run(config.evacuation_speed * direction * 0.4, -config.evacuation_speed * direction * 0.4, time_step)
                motors.run(0, 0, time_step)

            image = camera.capture_array()

            mask = image.copy()

            if config.victim_count < 2:
                mask = cv2.inRange(image, (10, 50, 10), (80, 160, 50))
                mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
            else:
                mask = cv2.inRange(image, (0, 0, 60), (40, 40, 255))
                mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
                
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                if config.X11: cv2.imshow("image", image)
                print("")
                continue

            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) < 2000:
                if config.X11: cv2.imshow("image", image)
                print("")
                continue

            x, y, w, h = cv2.boundingRect(largest_contour)

            direction = -1 if config.WIDTH/2 - int(x+ w/2) >= 0 else 1

            if config.X11:
                cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
                cv2.circle(image, (int(x + w / 2), int(y + h / 2)), 3, (255, 0, 0), 1)
                cv2.imshow("image", image)

            if config.WIDTH/2 - (x + w / 2) < tolerance and config.WIDTH/2 - (x + w / 2) > -tolerance: break

            print(f"{config.WIDTH/2 - (x + w / 2)}")

    print("(TRIANGLE SEARCH) Initial alignment")
    align(tolerance=10, text="Initial Alignment")

    motors.run(0, 0, 0.3)
    motors.run_until(config.evacuation_speed, config.evacuation_speed, laser_sensors.read, 1, "<=", 35)
    motors.run(0, 0, 0.3)
    motors.run_until(-config.evacuation_speed, -config.evacuation_speed, laser_sensors.read, 1, ">=", 35)

    print("(TRIANGLE SEARCH) Fine alignment")
    align(tolerance=3, text="Fine Alignment", time_step=0.1)

    motors.run(0, 0)