import time
import colour
import laser_sensors
import touch_sensors
import motors
import evacuation_zone

silver_min = 120
silver_count = 0
inverse_count = 0
inverse_threshold = 30
white_min = 60
black_max = 30
green_max = 20

outer_multi = 0.7
inner_multi = 0.7
line_speed = 20
line_ignore_value = 20
green_distance = 20
main_loop_count = green_distance

def follow_line():
    global main_loop_count, inverse_count

    main_loop_count += 1
    colour_values = colour.read(display_mapped=True)
    touch_values = touch_sensors.read(display=True)
    print()

    green_signal = check_green(colour_values)
    silver_count = silver_check(colour_values)
    if silver_count > 10:
        print("Silver Found")
        motors.run(0, 0, 1)

        evacuation_zone.main()

    elif green_signal is not None and inverse_count < 5 and main_loop_count > green_distance:
        align_black(green_signal)
        main_loop_count = 0
    elif touch_values[0] == 0 or touch_values[1] == 0:
        avoid_obstacle()
    else:
        turn = follow_black_line(colour_values, False)
        motors.run(turn[0], turn[1])

def align_black(align_type):
    global white_min, black_max
    turn_count = 0
    if align_type == 'left':
        print("Aligning from left.")
        motors.run(-line_speed - 5, line_speed + 5, 0.8)
        motors.run(line_speed, line_speed, 0.2)
        colour_values = colour.read()
        while colour_values[0] < black_max and turn_count < 500:
            turn_count += 1
            colour_values = colour.read(display_mapped=True)
            print()

    elif align_type == 'right':
        print("Aligning from right.")
        motors.run(line_speed + 5, -line_speed - 5, 0.8)
        motors.run(line_speed, line_speed, 0.2)
        colour_values = colour.read()
        while colour_values[4] < black_max and turn_count < 500:
            turn_count += 1
            colour_values = colour.read(display_mapped=True)
            print()
    
    elif align_type == 'double':
        print("Aligning from double.")
        motors.run(line_speed + 5, line_speed + 5, 0.2)
        motors.run(-line_speed - 10, line_speed + 10, 2)
        colour_values = colour.read()
        while colour_values[3] > white_min and turn_count < 1000:
            turn_count += 1
            colour_values = colour.read(display_mapped=True)
            print()

def check_inverse(colour_values):
    global black_max, inverse_count, inverse_threshold

    left_double_black = colour_values[0] < black_max or colour_values[1] < black_max and (colour_values[3] < black_max + 30 or colour_values[4] < black_max + 10)
    right_double_black = colour_values[3] < black_max or colour_values[4] < black_max and (colour_values[0] < black_max + 20 or colour_values[1] < black_max + 30)

    if (left_double_black or right_double_black) and ((colour_values[5] > 40 and colour_values[6] > 40) or (colour_values[5] < 10 and colour_values[6] < 10)):
        if inverse_count < inverse_threshold + 5:
            inverse_count += 1


    elif inverse_count > 0 and colour_values[0] > 50 and colour_values[1] > 50 and colour_values[3] > 50 and  colour_values[4] > 50:
        inverse_count = inverse_count // 10
        inverse_count -= 1

    
    print(f"Inverse Count: {inverse_count}", end=", ")

    return inverse_count
        
def follow_black_line(colour_values, inverse_following):
    global outer_multi, inner_multi, line_speed, line_ignore_value, inverse_count, inverse_threshold
    """
    Follows the black line.

    param colour_values: Mapped colour sensor values (index 0 to 7).
    param outer_multi: Factor weighs importance of outer sensor values.
    param inner_multi: Factor weighs importance of inner sensor values.
    param line_speed: Speed of car when going straight.
    param line_ignore_value: If there is an error lower than ignore value it will go straight to reduce jitter.
    return: An error which is used for turning values.
    """  

    outer_error = outer_multi * (colour_values[0] - colour_values[4])
    inner_error = inner_multi * (colour_values[1] - colour_values[3])

    if inverse_following:
        inverse_count = check_inverse(colour_values)

        if inverse_count > inverse_threshold:
            front_multi = 1 + (100 - colour_values[2]) / 100
            total_error = -0.3 * (outer_error + inner_error)
            print(f"Inverse Following", end=", ")
        else:
            front_multi = 1 + (colour_values[2] - 100) / 100
            total_error = outer_error + inner_error
    else:
        front_multi = 1 + (colour_values[2] - 80) / 100
        total_error = outer_error + inner_error

    if abs(total_error) < line_ignore_value:
        turn = [line_speed, line_speed]
    else:
        turn = [line_speed + front_multi * total_error, line_speed - front_multi * total_error]

    return turn

def check_green(colour_values):
    global black_max, green_max
    left_black = colour_values[0] < black_max and colour_values[1] < black_max
    right_black = colour_values[3] < black_max and colour_values[4] < black_max
    left_double_black = left_black and (colour_values[3] < black_max + 10 or colour_values[4] < black_max + 10)
    right_double_black = right_black and (colour_values[0] < black_max + 10 or colour_values[1] < black_max + 10)

    left_green = left_black and colour_values[5] < green_max
    right_green = right_black and colour_values[6] < green_max
    left_double_green = left_green and colour_values[6] < green_max + 10
    right_double_green =  right_green and colour_values[5] < green_max + 10
    double_green = (left_double_black and left_double_green) or (right_double_black and right_double_green)

    if double_green:
        return 'double'
    elif left_green:
        return 'left'
    elif right_green:
        return 'right'

def circle_obstacle(laser_pin, turn_type, more_than, check_black, colour_is_black, min_move_time):
    global black_max
    laser_condition_met = False
    starting_time = time.time()
    current_time = 0

    while (not laser_condition_met or current_time < min_move_time) and (not colour_is_black[0] or not colour_is_black[1]):
        if turn_type == 'straight':
            motors.run(30, 30)
        elif turn_type == 'left':
            motors.run(25, -25)
        elif turn_type == 'right':
            motors.run(-25, 25)

        colour_values = colour.read(display_mapped=True)
        distances = laser_sensors.read(display=True)
        touch_values = touch_sensors.read(display=True)
        print()

        if more_than == '>':
            laser_condition_met = distances[laser_pin] > 10
        elif more_than == '<':
            laser_condition_met = distances[laser_pin] < 10

        if check_black:
            colour_is_black[0] = (colour_values[1] < black_max + 20 and not colour_is_black[0]) or colour_is_black[0]
            colour_is_black[1] = (colour_values[3] < black_max + 20 and not colour_is_black[1]) or colour_is_black[1]

        for touch_side in range(1):
            if touch_values[touch_side] == 0 and turn_type == 'straight':
                if touch_side == 0:
                    motors.run(line_speed, -line_speed)
                if touch_side == 1:
                    motors.run(-line_speed, line_speed)

                colour.read(display_mapped=True)
                laser_sensors.read(display=True)
                touch_sensors.read(display=True)
                print()

        print(f"Laser Condition Met: {laser_condition_met}")

        current_time = time.time() - starting_time

    return colour_is_black

def avoid_obstacle():
    global line_speed
    print("Obstacle Detected")
    colour_is_black = [False, False]

    distances = laser_sensors.read(display=True)
    print()

    if distances[0] > distances[2]:
        print("Going Left")
        colour_is_black = circle_obstacle(2, 'right', '<', False, colour_is_black, 0.5)
        colour_is_black = circle_obstacle(2, 'straight', '>', False, colour_is_black, 0.5)

        print("Left Obstacle Loop Phase")
        while not colour_is_black[0]:
            colour_is_black = circle_obstacle(2, 'left', '>', True, colour_is_black, 1)
            colour_is_black = circle_obstacle(2, 'straight', '>', True, colour_is_black, 1)

        align_black('left')
    else:
        print("Going Right")
        colour_is_black = circle_obstacle(0, 'left', '<', False, colour_is_black, 0.5)
        colour_is_black = circle_obstacle(0, 'straight', '>', False, colour_is_black, 0.5)

        print("Right Obstacle Loop Phase")
        while not colour_is_black[1]:
            colour_is_black = circle_obstacle(0, 'right', '>', True, colour_is_black, 1)
            colour_is_black = circle_obstacle(0, 'straight', '>', True, colour_is_black, 1)

        align_black('right')


def silver_check(colour_values):
    global silver_min, silver_count
    if (colour_values[0] > silver_min or colour_values[1] > silver_min or colour_values[3] > silver_min or colour_values[4] > silver_min) and colour_values[2] > black_max:
        silver_count += 1
    
    return silver_count
    