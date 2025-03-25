import cv2
import numpy as np
import config
import laser_sensors
import touch_sensors
import camera
import motors

def find() -> None:
    config.update_log(["TRIANGLE", "initial alignment"], [24, 24])
    print()
    align(tolerance=10, text="Initial Alignment")
    
    move_closer(1)
    motors.run      ( config.evacuation_speed * 0.8,  config.evacuation_speed * 0.8, 0.3)
    motors.run_until(-config.evacuation_speed * 0.8, -config.evacuation_speed * 0.8, laser_sensors.read, 1, ">=", 25, "STANDARDIZING DISTANCE")

    config.update_log(["TRIANGLE", "fine alignment"], [24, 24])
    print()
    align(tolerance=3, text="Fine Alignment", time_step=0.05)
        
    motors.run(0, 0)

def locate(image: np.ndarray) -> tuple[int, int, int, int]:
    # Setup
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = hsv_image.copy()

    # Determine filter
    if config.victim_count < 2:
        # Adjusting: H_max | H_min=35, H_max=100, S_min=100, S_max=255, V_min=10, V_max=255
        mask = cv2.inRange(hsv_image, (35, 100, 10), (100, 255, 255))
        mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
    else:
        # Adjusting: V_min | H_min=0, H_max=10, S_min=230, S_max=255, V_min=46, V_max=255
        # Adjusting: S_min | H_min=170, H_max=179, S_min=230, S_max=255, V_min=0, V_max=255
        mask_lower = cv2.inRange(hsv_image, (0, 230, 46), (10, 255, 255))
        mask_upper = cv2.inRange(hsv_image, (170, 230, 0), (179, 255, 255))
        mask = cv2.bitwise_or(mask_lower, mask_upper)
        mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
        
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If no contours are found, return None
    if not contours: return None, None, None, None
    largest_contour = max(contours, key=cv2.contourArea)

    # If the largest contour is too small, return None
    if cv2.contourArea(largest_contour) < 1200: return None, None, None, None

    # Get the bounding rectangle of the largest contour
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Display debug information
    if config.X11:
        cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)
        cv2.circle(image, (int(x + w / 2), int(y + h / 2)), 3, (255, 0, 0), 1)
        cv2.imshow("image", image)

    return x, y, w, h

def align(tolerance: int, text: str, time_step: float = None) -> None:
    # Initialise directin to clockwise
    direction = 1

    while True:
        image = camera.capture_array()
        distance = laser_sensors.read([config.x_shut_pins[1]])[0]

        # Determine movement type
        if time_step is None:
            motors.run(config.evacuation_speed * direction * 0.5, -config.evacuation_speed * direction * 0.5)      
        else:
            motors.run(config.evacuation_speed * direction * 0.5, -config.evacuation_speed * direction * 0.5, time_step)
            motors.run(0, 0, time_step)

        # Determine movement direction
        if distance is None:
            pass

        elif distance < 15:
            motors.run_until(-config.evacuation_speed, -config.evacuation_speed, laser_sensors.read, 1, ">=", 15, "STANDARDIZING DISTANCE")

        x, _, w, _ = locate(image)

        if x is None: continue

        offset = 15

        direction = -1 if config.EVACUATION_WIDTH/2 - int(x+ w/2) + offset >= 0 else 1

        if config.EVACUATION_WIDTH/2 - (x + w / 2)  + offset < tolerance and config.EVACUATION_WIDTH/2 - (x + w / 2)  + offset > -tolerance: break

        config.update_log([f"{text}", "green" if config.victim_count < 2 else "red", f"{config.EVACUATION_WIDTH/2 - (x + w / 2)}"], [24, 10, 10])
        print()

def move_closer(kP: float) -> None:
    while True:
        image = camera.capture_array()
        touch_values = touch_sensors.read()

        x, _, w, _ = locate(image)

        print(x, w)
        print(x is None, w is None)

        if x is None and w is None:
            print("X IS NONE!")
            motors.run(0, 0)
            continue
        
        error = int(config.EVACUATION_WIDTH/2 - (x + w/2))
        turn = error * kP

        v1, v2 = config.evacuation_speed - turn, config.evacuation_speed + turn
        motors.run(v1, v2)

        if sum(touch_values) != 2:
            break

        config.update_log([f"MOVING CLOSER", "green" if config.victim_count < 2 else "red", f"{error}"], [24, 10, 10])
        print()