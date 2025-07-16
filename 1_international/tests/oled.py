import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

# from core.utilities import *
# from hardware.robot import *
# from core.shared_imports import time

# oled_display.clear()
# oled_display.draw_circle(64, 32, 40, 7, update_display=True)
# F_size = 30
# oled_display.text("F", int(64 - F_size / 4) + 1, int(32 - F_size / 2) - 3, size=F_size, font_family="cambria", update_display=True)

# # time.sleep(1)
# # oled_display.draw_spiral_cutouts(64, 32, 39, 49, 5, update_display=True)
# # time.sleep(1)


# # time.sleep(1)

from core.shared_imports import time
from hardware.robot import oled_display
from core.utilities import start_display

def test_oled_display():
    start_display()

    messages = [
        # ("SILVER", 8, 12, 30, True),
        # ("RED", 35, 12, 30, True),
        # ("OBSTACLE", 15, 18, 20, True),
        # ("SEESAW", 8, 12, 30, True),
        # ("GAP", 38, 12, 30, True),
        # ("LINE", 25, 12, 30, True),
        # ("STUCK", 20, 12, 30, True),

        ("GREEN", 15, 0, 30, True),
        ("LEFT", 25, 30, 30, False),

        ("GREEN", 15, 0, 30, True),
        ("RIGHT", 15, 30, 30, False),

        ("GREEN", 15, 0, 30, True),
        ("DOUBLE", 8, 30, 30, False),
    ]


    for text, x, y, size, clear in messages:
        oled_display.text(text, x, y, size=size, clear=clear)
        print(f"Displaying: {text}")
        time.sleep(5)

    oled_display.text("DONE", 20, 30, size=20, clear=True)
    print("OLED Test Complete.")

if __name__ == "__main__":
    test_oled_display()
