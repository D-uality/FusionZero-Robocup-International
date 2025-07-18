from core.shared_imports import Path, time, randint, operator, np
from core.utilities import user_at_host, debug, debug_lines, start_display, show
from core.listener import listener
from hardware.robot import *
import behaviours.optimized_evacuation as evacuation_zone
from behaviours.silver_detection import SilverLineDetector

current_dir = Path(__file__).resolve().parent
modules_dir = current_dir / 'modules'


if user_at_host == "frederick@raspberrypi":
    silver_detector = SilverLineDetector("/home/frederick/FusionZero-Robocup-International/5_ai_training_data/0_models/silver_line/silver_detector_pi4_quantized.pt")
else:
    silver_detector = SilverLineDetector("/home/aidan/FusionZero-Robocup-International/5_ai_training_data/0_models/silver_line/silver_detector_pi4_quantized.pt")

start_display()

def main(start_time, robot_state, line_follow) -> None:
    robot_state.debug_text.clear()
    overall_start = time.perf_counter()
    led.on()

    touch_values = touch_sensors.read()
    gyro_values = gyroscope.read()
    silver_value = silver_sensor.read()
    touch_check(robot_state, touch_values)
    ramp_check(robot_state, gyro_values)
    update_tilt_triggers(robot_state)
    find_silver(robot_state, line_follow, silver_detector, silver_value)
    line_follow.find_red()

    if time.perf_counter() - start_time < 3:
        while (time.perf_counter() - start_time < 3) and listener.mode.value != 0:
            motors.run(0, 0)
            line_follow.follow(starting=True)

    elif robot_state.count["silver"] >= 3:
        motors.run(25, -25, 0.3)
        oled_display.text("SILVER", 8, 12, size=30, clear=True)
        print("Silver Found!")
        robot_state.count["silver"] = 0
        motors.run(0, 0)
        evacuation_zone.main()
        motors.run(-30, -30, 0.3)
        line_follow.align_to_contour_angle()
        print("finished align")
        # robot_state.trigger["evacuation_zone"] = True
        motors.run(0, 0, 1)
        
        start_time = time.perf_counter()
        led.on()
        
        
    elif robot_state.count["red"] >= 5:
        oled_display.text("RED", 35, 12, size=30, clear=True)
        print("Red Found!")
        motors.run(0, 0, 10)
        robot_state.count["red"] = 0

    elif robot_state.count["touch"] > 10:
        oled_display.text("OBSTACLE", 15, 18, size=20, clear=True)
        print("Obstacle Detected")
        robot_state.count["touch"] = 0
        avoid_obstacle(line_follow, robot_state)

    else:
        line_follow.follow()

    active_triggers = ["LINE"]
    for key in robot_state.trigger:
        if robot_state.trigger[key]: active_triggers.append(key)

    total_elapsed = time.perf_counter() - overall_start
    fps = int(1.0 / total_elapsed) if total_elapsed > 0 else 0
    robot_state.debug_text.append(f"FPS: {fps}")
    
    robot_state.debug_text.insert(0, f"{active_triggers}")
    if robot_state.debug: debug_lines(robot_state.debug_text)
    robot_state.main_loop_count += 1

# ========================================================================
# SILVER
# ========================================================================

def find_silver(robot_state, line_follow, silver_detector, silver_value) -> None:
    # if silver_value > 120:
    robot_state.last_seen_silver = time.perf_counter()

    if line_follow.black_mask is not None:
        top_mask = line_follow.black_mask[:int(camera.LINE_HEIGHT / 8), :]
        top_line = np.any(top_mask)
    else:
        top_line = True

    if line_follow.image is not None and not top_line and time.perf_counter() - robot_state.last_seen_silver < 2 and not line_follow.green_signal:
        result = silver_detector.predict(line_follow.image)
        robot_state.debug_text.append(f"SILVER: {result['class_name']}, {result['confidence']:.3f}")
        robot_state.count["silver"] = robot_state.count["silver"] + 1 if result['prediction'] == 1 and result['confidence'] > 0.99 else 0
        
# ========================================================================
# TILT
# ========================================================================  

def ramp_check(robot_state, gyro_values: list[int]) -> None:
    if gyro_values is not None:
        pitch = gyro_values[0]
        roll  = gyro_values[1]
        
        robot_state.count["uphill"]     = robot_state.count["uphill"]     + 1 if pitch >=  10 else 0
        robot_state.count["downhill"]   = robot_state.count["downhill"]   + 1 if pitch <= -10 else 0
        
        robot_state.count["tilt_left"]  = robot_state.count["tilt_left"]  + 1 if roll  <= -10 else 0
        robot_state.count["tilt_right"] = robot_state.count["tilt_right"] + 1 if roll  >= 10 else 0
        
        robot_state.last_uphill = 0 if robot_state.trigger["uphill"] else robot_state.last_uphill + 1
        robot_state.last_downhill = 0 if robot_state.count["downhill"] > 0 else robot_state.last_downhill + 1

def update_tilt_triggers(robot_state) -> list[str]:
    prev_triggers = robot_state.trigger

    if robot_state.count["uphill"] > 5:
        robot_state.trigger["uphill"] = True
    else:
        robot_state.trigger["uphill"] = False
         
    if robot_state.count["downhill"] > 5:
        robot_state.trigger["downhill"] = True
        if robot_state.prev_downhill == False:
            robot_state.time_since_downhill = time.perf_counter()
    else:
        robot_state.trigger["downhill"] = False
        
    robot_state.prev_downhill = robot_state.trigger["downhill"]
         
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

# ========================================================================
# OBSTACLE
# ========================================================================

def touch_check(robot_state, touch_values: list[int]) -> None:
    robot_state.count["touch"] = robot_state.count["touch"] + 1 if sum(touch_values) != 2 else 0

def avoid_obstacle(line_follow, robot_state) -> None:
    # if robot_state.trigger["downhill"] or robot_state.trigger["uphill"] or robot_state.trigger["tilt_left"] or robot_state.trigger["tilt_right"]:
    touch_count = 0
    
    while listener.mode.value != 0:
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

    right_value = laser_sensors.read([2])[0]
    left_value = laser_sensors.read([0])[0]

    # Clockwise if left > right
    left_value = 255 if left_value is None else left_value
    right_value= 255 if right_value is None else right_value
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
    # oled_display.text("Turning till obstacle", 0, 10, size=10)

    if robot_state.count["downhill"] > 0:
        motors.run(-30-10, -30-10, 1)
        print("Backing up")
    elif robot_state.count["uphill"] > 5:
        motors.run(-30, -30, 0.2)
    else:
        motors.run(-30, -30, 0.4)

    for i in range(3):
        if robot_state.count["downhill"] > 0:
            motors.run_until(v1, v2, laser_sensors.read, 1, ">=", 9.5, "TURNING PAST OBSTACLE (MIDDLE)")
        else:
            motors.run_until(v1, v2, laser_sensors.read, 1, ">=", 9.5, "TURNING PAST OBSTACLE (MIDDLE)")
        motors.run(v1, v2, 0.01)

    if robot_state.count["uphill"] > 5:
        motors.run(v1, v2, 0.5)
    else:
        motors.run(v1, v2, 1)

    # Circle obstacle
    v1 =  30 if direction == "cw" else -30
    v2 = -30 if direction == "cw" else 30
    laser_pin = 2 if direction == "cw" else 0
    colour_pin = 2

    initial_sequence = True

    circle_obstacle(30, 30, laser_pin, colour_pin, "<=", 10, "FORWARDS TILL OBSTACLE", initial_sequence, direction)

    while listener.mode.value != 0:
        if circle_obstacle(30, 30, laser_pin, colour_pin, ">=", 18, "FORWARDS TILL NOT OBSTACLE", False, direction): pass
        elif not initial_sequence: break
        if robot_state.count["uphill"] > 1:
            motors.run(30, 30, 0.1)
        elif robot_state.last_downhill < 100:
            motors.run(-30, -30, 0.3)
        motors.run(0, 0, 0.15)

        if circle_obstacle(v1, v2, laser_pin, colour_pin, "<=", 10, "TURNING TILL OBSTACLE", False, direction): pass
        elif not initial_sequence: break
        motors.run(0, 0, 0.15)

        if circle_obstacle(v1, v2, laser_pin, colour_pin, ">=", 20, "TURNING TILL NOT OBSTACLE", False, direction): pass
        elif not initial_sequence: break
        motors.run(v1, v2, 0.4)
        motors.run(0, 0, 0.15)

        if circle_obstacle(30, 30, laser_pin, colour_pin, "<=", 13, "FORWARDS TILL OBSTACLE", False, direction): pass
        elif not initial_sequence: break
        motors.run(0, 0, 0.15)

        initial_sequence = False

    # oled_display.reset()
    # oled_display.text("Black Found", 0, 0, size=10)
    debug(["OBSTACLE", "FOUND BLACK"], [24, 50])

    if robot_state.count["downhill"] < 5: motors.run(30, 30, 0.2)
    else: 
        print("Backing Up")
        motors.run(-30, -30, 0.5)
    
    while True:
        gyro_value = gyroscope.read()
        if gyro_value is not None:
            break
    
    pitch, _, _ = gyro_value
    
    print(pitch)
        
    if pitch > 15: loops = 1
    else: loops = 3

    if pitch < -15:
        motors.run(-v1, -v2, 2.3)
        motors.run(-30, -30, 0.5)
        motors.run(0, 0, 2)

    for i in range(loops):
        if direction == "cw":
            line_follow.run_till_camera(-v1, -v2-5, 20, ["OBSTACLE"])
        else:
            line_follow.run_till_camera(-v1-5, -v2, 20, ["OBSTACLE"])
    
def circle_obstacle(v1: float, v2: float, laser_pin: int, colour_pin: int, comparison: str, target_distance: float, text: str = "", initial_sequence: bool = False, direction: str = "") -> bool:
    if   comparison == "<=": comparison_function = operator.le
    elif comparison == ">=": comparison_function = operator.ge

    while listener.mode.value != 0:
        show(camera.perspective_transform(camera.capture_array()), display=camera.X11, name="line", debug_lines=["OBSTACLE"])
        laser_value = laser_sensors.read([laser_pin])[0]
        colour_values = colour_sensors.read()
        touch_values = touch_sensors.read()

        debug(["OBSTACLE", text, f"{laser_value}"], [24, 50, 10])

        if laser_value is None: laser_value = 255

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