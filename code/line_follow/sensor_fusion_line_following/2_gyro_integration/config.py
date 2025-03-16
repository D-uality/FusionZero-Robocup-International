EVACUATION_WIDTH, EVACUATION_HEIGHT, FLIP = 320, 240, False
LINE_WIDTH, LINE_HEIGHT = 160, 90
SCREEN_WIDTH, SCREEN_HEIGHT = 128, 64
X11 = True

touch_pins = [6, 26]  # FL, FR
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

line_speed = 22