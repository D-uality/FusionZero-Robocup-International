import sensors
import motors

white_min = 60
black_max = 20
green_max = -20

outer_multi = 1.1
inner_multi = 1.1
line_speed = 25
line_ignore_value = 10
green_distance = 20
main_loop_count = green_distance

def follow_line():
    global main_loop_count

    main_loop_count += 1
    colour_values = sensors.read_mapped_analog(True)

    green_signal = check_green(colour_values, black_max, green_max)
    if green_signal is not None and main_loop_count > green_distance:
        align_black(green_signal, white_min, black_max)
        main_loop_count = 0
    else:
        turn = follow_black_line(colour_values, outer_multi, inner_multi, line_speed, line_ignore_value)
        motors.run(turn[0], turn[1])

def align_black(align_type, white_min, black_max):
    turn_count = 0
    if align_type == 'left':
        print("Aligning from left.")
        motors.run(-line_speed - 5, line_speed + 5, 0.7)
        motors.run(line_speed, line_speed)
        colour_values = sensors.read_mapped_analog(True)
        while colour_values[0] < black_max and turn_count < 50:
            turn_count += 1
            colour_values = sensors.read_mapped_analog(True)

    elif align_type == 'right':
        print("Aligning from right.")
        motors.run(line_speed + 5, -line_speed - 5, 0.7)
        motors.run(line_speed, line_speed)
        colour_values = sensors.read_mapped_analog(True)
        while colour_values[4] < black_max and turn_count < 50:
            turn_count += 1
            colour_values = sensors.read_mapped_analog(True)
    
    elif align_type == 'double':
        print("Aligning from double.")
        motors.run(line_speed + 5, line_speed + 5, 0.3)
        motors.run(-line_speed - 10, line + 10, 0.8)
        colour_values = sensors.read_mapped_analog(True)
        while colour_values[3] > white_min and turn_count < 300:
            turn_count += 1
            colour_values = sensors.read_mapped_analog(True)


def follow_black_line(colour_values, outer_multi, inner_multi, line_speed, line_ignore_value):
    """
    Follows the black line.

    param colour_values: Mapped colour sensor values (index 0 to 7).
    param outer_multi: Factor weighs importance of outer sensor values.
    param inner_multi: Factor weighs importance of inner sensor values.
    param line_speed: Speed of car when going straight.
    param line_ignore_value: If there is an error lower than ignore value it will go straight to reduce jitter.
    return: An error which is used for turning values.
    """  

    front_multi = 1 + (colour_values[2] - 100) / 100

    outer_error = outer_multi * (colour_values[0] - colour_values[4])
    inner_error = inner_multi * (colour_values[1] - colour_values[3])

    total_error = outer_error + inner_error
    if abs(total_error) < line_ignore_value:
        turn = [line_speed, line_speed]
    else:
        turn = [line_speed + front_multi * total_error, line_speed - front_multi * total_error]

    return turn

def check_green(colour_values, black_max, green_max):
    left_black = colour_values[0] < black_max and colour_values[1] < black_max and colour_values[2] < black_max
    right_black = colour_values[3] < black_max and colour_values[4] < black_max and colour_values[2] < black_max
    left_double_black = left_black and colour_values[3] < black_max + 10 and colour_values[4] < black_max + 10
    right_double_black = right_black and colour_values[0] < black_max + 10 and colour_values[1] < black_max + 10

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