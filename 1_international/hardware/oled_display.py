from core.shared_imports import board, Image, ImageDraw, ImageFont, adafruit_ssd1306
from core.utilities import debug


class OLED_Display:
    def __init__(self):
        self.HEIGHT = 64
        self.WIDTH = 128
        self.font_cache: dict[int, ImageFont.ImageFont] = {}

        try:
            i2c = board.I2C()
        except Exception as e:
            raise RuntimeError("I2C bus unavailable") from e

        self.oled = adafruit_ssd1306.SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c, addr=0x3C)
        self.reset()
        debug(["INITIALISATION", "OLED", "✓"], [25, 25, 50])

    def reset(self) -> None:
        self.oled.fill(0)
        self.oled.show()
        self.image = Image.new("1", (self.WIDTH, self.HEIGHT))

    def _font(self, size: int) -> ImageFont.ImageFont:
        if size not in self.font_cache:
            self.font_cache[size] = ImageFont.truetype(
                "/home/frederick/FusionZero-Robocup-International/1_international/hardware/fonts/JetBrainsMono-Regular.ttf",
                size=size,
            )
        return self.font_cache[size]

    def text(self, text: str, x: int, y: int, size: int = 10) -> None:
        image = Image.new("1", (self.WIDTH, self.HEIGHT))
        draw = ImageDraw.Draw(image)
        draw.text((x, y), str(text), fill=255, font=self._font(size))
        self.oled.image(image)
        self.oled.show()
