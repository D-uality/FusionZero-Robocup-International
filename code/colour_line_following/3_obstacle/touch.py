import RPi.GPIO as GPIO

touch_pins = [5, 6, 22, 26]  # FL, FR, BL, BR

def init() -> None:
    try:
        for touch_pin in touch_pins:
            GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        print(["Touch", "✓"])
        
    except Exception as e:
        print(["Touch", "X", f"{e}"])

def read(pins=touch_pins, display=False):
    values = []

    for pin in pins:
        values.append(GPIO.input(pin))

    if display:
        print(f"T: {values}", end=", ")

    return values