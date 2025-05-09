from config import *
import numpy as np
import cv2

import camera
import motors
import laser_sensors

def find(image):
    kernal_size = 7
    spectral_highlights = cv2.inRange(image, (200, 200, 200), (255, 255, 255))
    spectral_highlights = cv2.dilate(spectral_highlights, np.ones((kernal_size, kernal_size), np.uint8), iterations=1)

    contours, _ = cv2.findContours(spectral_highlights, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours: return None

    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)

    if X11: cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 1)

    return int(x + w/2)

def route(base_speed, kP, target_distance):
    distance = laser_sensors.read([x_shut_pins[1]])
    while distance is None: distance = laser_sensors.read([x_shut_pins[1]])

    while distance[0] > target_distance:
        image = camera.capture_array()
        distance = laser_sensors.read([x_shut_pins[1]])
        x = find(image, 7)

        if x is None: return False

        scalar = 0.5 + (distance[0] - 15) * (0.5 / 85)
        error = WIDTH / 2 - x
        turn = int(error * kP)
        v1, v2 = scalar * (base_speed-turn), scalar * (base_speed+turn)
        motors.run(v1, v2)

        cv2.imshow("image", image)
        print(f"(APPROACHING)    |    {error=} {scalar=:.2f} {turn=}")

    motors.run(0, 0, 0.5)
    motors.run_until(-base_speed * 0.62, -base_speed * 0.62, laser_sensors.read, 1, ">=", target_distance, "ROUTE BACK")

    return True

def align(base_speed, target_distance):
    print("(ALIGNING)")
    image = camera.capture_array()
    x = find(image, 7)
    if x is None: return False
    error = WIDTH / 2 - x

    while error > 2:
        image = camera.capture_array()
        x = find(image, 7)
        if x is None: return False
        
        error = WIDTH / 2 - x
        motors.run(-base_speed * 0.62, base_speed * 0.62, 0.005)
        motors.run(0, 0, 0.01)

        if X11: cv2.imshow("image", image)
        print(f"(Aligning Right)    |    {error=}")

    motors.run(0, 0, 0.5)

    while error < -2:
        image = camera.capture_array()
        x = find(image, 7)
        if x is None: return False

        error = WIDTH / 2 - x
        motors.run(base_speed * 0.62, -base_speed * 0.62, 0.005)
        motors.run(0, 0, 0.01)
    
        if X11: cv2.imshow("image", image)
        print(f"(Aligning Left)    |    {error=}")

    motors.run(0, 0, 0.3)
    motors.run_until(-base_speed * 0.62, -base_speed * 0.62, laser_sensors.read, 1, ">=", target_distance, "ALIGN BACK")
    motors.run_until( base_speed * 0.62,  base_speed * 0.62, laser_sensors.read, 1, "<=", target_distance, "ALIGN FRONT")

    return True