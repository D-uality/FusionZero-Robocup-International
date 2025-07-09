from core.shared_imports import board, Image, ImageDraw, ImageFont, adafruit_ssd1306
from core.utilities import debug

class OLED_Display():
    def __init__(self):
        self.HEIGHT = 64
        self.WIDTH = 128

        i2c = board.I2C()

        self.oled = adafruit_ssd1306.SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c, addr=0x3c)
        self.image = Image.new("1", (self.WIDTH, self.HEIGHT))

        self.oled.fill(0)
        self.oled.show()

        debug(["INITIALISATION", "OLED", "✓"], [25, 25, 50])

    def reset(self) -> None:
        self.oled.fill(0)
        self.oled.show()

    def text(self, text: str, x: int, y: int, size: int = 10) -> None:
        font = ImageFont.truetype("/home/fusion/.fonts/JetBrainsMono-Regular.ttf", size=size)

        draw = ImageDraw.Draw(self.image)
        draw.text((x, y), str(text), fill="white", font=font)

        self.oled.image(self.image)
        self.oled.show()