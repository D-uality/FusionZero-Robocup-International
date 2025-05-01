import adafruit_ssd1306
import board
from PIL import Image, ImageDraw, ImageFont
import config

i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(config.SCREEN_WIDTH, config.SCREEN_HEIGHT, i2c, addr=0x3c)
image = Image.new("1", (config.SCREEN_WIDTH, config.SCREEN_HEIGHT))

def initialise() -> None:
    try:
        oled.fill(0)
        oled.show()

        config.update_log(["INITIALISATION", "OLED", "✓"], [24, 15, 50])
        print()

    except Exception as e:
        config.update_log(["INITIALISATION", "OLED", f"{e}"], [24, 15, 50])
        print()

def reset() -> None:
    global image
    image = Image.new("1", (config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    oled.fill(0)
    oled.show()

def text(text: str, x: int, y: int, size: int = 10) -> None:
    font = ImageFont.truetype("/home/fusion/.fonts/JetBrainsMono-Regular.ttf", size=size)

    draw = ImageDraw.Draw(image)

    draw.text((x, y), str(text), fill="white", font=font)

    oled.image(image)
    oled.show()