import sensors
import motors

outer_multi = 0.6
inner_multi = 0.6
line_speed = 20

def follow_line():
    colour_values = sensors.read_mapped_analog(True)

    turn = follow_black_line(colour_values)
    motors.run(turn[0], turn[1])

def follow_black_line(colour_values):
    """
    Follows the black line.

    param colour_values: Front mapped colour sensor values (index 0 to 4).
    return: An error which is used for turning values
    """  

    outer_error = outer_multi * (colour_values[0] - colour_values[3])
    inner_error = inner_multi * (colour_values[1] - colour_values[2])

    total_error = int(outer_error + inner_error)
    turn = [ line_speed + total_error, line_speed - total_error ]

    return turn