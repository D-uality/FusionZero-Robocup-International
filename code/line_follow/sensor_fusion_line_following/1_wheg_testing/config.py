WIDTH, HEIGHT, FLIP = 320, 240, False
SCREEN_WIDTH, SCREEN_HEIGHT = 128, 64
X11 = False

touch_pins = [5, 6, 22, 26]  # FL, FR, BL, BR
x_shut_pins = [23, 24, 25]  # Left, Middle, Right

stop_angles = [88, 89, 88, 89]
servo_pins = [14, 13, 12, 10]  # FL, FR, BL, BR
claw_pin = 9

def update_log(data: list[str], coloumn_widths: list[int], separator: str = "|"):
    formatted_cells = [f"{cell:^{width}}" for cell, width in zip(data, coloumn_widths)]
    print(f" {separator} ".join(formatted_cells), end="")

# ------------------------------------------

status_messages = []

victim_count = 0
evacuation_speed = 35
approach_distance = 18


# ------------------------------------------

outer_multi, inner_multi = 1, 1.3
line_base_speed = 15