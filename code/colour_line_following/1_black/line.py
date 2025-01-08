import sensors
import motors

def follow_line():
    colour_values = sensors.read_mapped_analog(True)

    turn = follow_black_line(colour_values)
    motors.run(turn[0], turn[1])

outer_multi = 1.1
inner_multi = 1.1
line_speed = 25
line_ignore_value = 10

def follow_black_line(colour_values):
    """
    Follows the black line.

    param colour_values: Mapped colour sensor values (index 0 to 7).
    return: An error which is used for turning values
    """  

    front_multi = 1 + (colour_values[2] - 100) / 100

    outer_error = outer_multi * (colour_values[0] - colour_values[4])
    inner_error = inner_multi * (colour_values[1] - colour_values[3])

    total_error = outer_error + inner_error
    if abs(total_error) < 10:
        turn = [line_speed, line_speed]
    else:
        turn = [line_speed + front_multi * total_error, line_speed - front_multi * total_error]

    return turn