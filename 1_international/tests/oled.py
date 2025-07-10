from PIL import Image, ImageDraw, ImageFont
import board, busio, adafruit_ssd1306, time

W, H = 128, 64
i2c  = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(W, H, i2c, addr=0x3C)

img  = Image.new("1", (W, H))
draw = ImageDraw.Draw(img)
draw.text((0, 0), "HELLO", font=ImageFont.load_default(), fill=255)
oled.image(img)
oled.rotate(True)
oled.show()
time.sleep(5)
oled.fill(0)
oled.rotate(True)
oled.show()
