from core.shared_imports import board, Image, ImageDraw, ImageFont, adafruit_ssd1306
from core.utilities import debug, user_at_host
import os


class OLED_Display:
    def __init__(self):
        self.HEIGHT = 64
        self.WIDTH = 128
        self.LOGO_PATH = r"/home/frederick/FusionZero-Robocup-International/4_documents/Fusion Zero Logo.png"
        
        # Font paths - more maintainable approach
        self.FONT_PATHS = {
            "jetbrains": {
                "frederick": "/home/frederick/FusionZero-Robocup-International/1_international/hardware/fonts/JetBrainsMono-Regular.ttf",
                "aidan": "/home/aidan/FusionZero-Robocup-International/1_international/hardware/fonts/JetBrainsMono-Regular.ttf"
            },
            "cambria": "/home/frederick/FusionZero-Robocup-International/1_international/hardware/fonts/Cambria.ttf"
        }
        
        self.font_cache: dict[str, ImageFont.ImageFont] = {}
        
        i2c = board.I2C()
        self.oled = adafruit_ssd1306.SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c, addr=0x3C)
        self.clear()
        
        debug(["INITIALISATION", "OLED", "✓"], [25, 25, 50])
        
    def _get_font_path(self, font_family: str) -> str:
        """Get the correct font path based on font family."""
        if font_family == "jetbrains":
            user = "frederick" if user_at_host == "frederick@raspberrypi" else "aidan"
            return self.FONT_PATHS["jetbrains"][user]
        
        elif font_family == "cambria":
            return self.FONT_PATHS["cambria"]
        
        else:
            raise ValueError(f"Unknown font family: {font_family}")
    
    def _font(self, size: int, font_family: str = "jetbrains") -> ImageFont.ImageFont:
        cache_key = f"{font_family}_{size}"
        
        if cache_key not in self.font_cache:
            try:
                font_path = self._get_font_path(font_family)
                if os.path.exists(font_path): self.font_cache[cache_key] = ImageFont.truetype(font_path, size=size)
                
                else:
                    self.font_cache[cache_key] = ImageFont.load_default()
                    debug(["FONT", "FALLBACK", f"{font_family} not found"], [25, 25, 50])
                    
            except Exception as e:
                self.font_cache[cache_key] = ImageFont.load_default()
                debug(["FONT", "ERROR", str(e)], [25, 25, 50])
        
        return self.font_cache[cache_key]

    def draw_circle(self, center_x: int, center_y: int, inner_diameter: int, thickness: int, fill_color: int = 255, update_display: bool = True) -> None:
        """Draw a circle with specified inner diameter and thickness."""
        # Calculate radii
        inner_radius = inner_diameter // 2
        outer_radius = inner_radius + thickness
        
        # Draw the outer circle (filled)
        self.draw.ellipse([
            center_x - outer_radius,
            center_y - outer_radius,
            center_x + outer_radius,
            center_y + outer_radius
        ], fill=fill_color)
        
        # Draw the inner circle (hollow it out)
        if inner_radius > 0:
            self.draw.ellipse([
                center_x - inner_radius,
                center_y - inner_radius,
                center_x + inner_radius,
                center_y + inner_radius
            ], fill=0)
        
        if update_display:
            self.oled.image(self.image)
            self.oled.show()
        
    def draw_spiral_cutouts(self, center_x: int, center_y: int, inner_radius: int, outer_radius: int, num_segments: int = 8, update_display: bool = True) -> None:        
        # Calculate angle per segment
        angle_per_segment = 360 / num_segments
        
        for i in range(num_segments):
            if i % 2 == 0:  # Cut out every other segment
                start_angle = i * angle_per_segment
                end_angle = (i + 1) * angle_per_segment
                
                # Draw a pie slice to cut out (fill with black)
                self.draw.pieslice([
                    center_x - outer_radius,
                    center_y - outer_radius,
                    center_x + outer_radius,
                    center_y + outer_radius
                ], start=start_angle, end=end_angle, fill=1)
                
                # Make sure we don't cut into the inner circle
                if inner_radius > 0:
                    self.draw.ellipse([
                        center_x - inner_radius,
                        center_y - inner_radius,
                        center_x + inner_radius,
                        center_y + inner_radius
                    ], fill=0)
        
        if update_display:
            self.oled.image(self.image)
            self.oled.show()

    def text(self, text: str, x: int, y: int, size: int = 15, font_family: str = "jetbrains", update_display: bool = True) -> None:
        chosen_font = self._font(size, font_family)
        self.draw.text((x, y), str(text), fill=255, font=chosen_font)
        
        if update_display:
            self.oled.image(self.image)
            self.oled.show()
            
    def display_logo(self):
        self.clear()

        self.draw_circle(64, 32, 40, 7, update_display=True)
        F_size = 30
        self.text("F", int(64 - F_size / 4) + 1, int(32 - F_size / 2) - 3, size=F_size, font_family="cambria", update_display=True)

    def clear(self) -> None:
        self.image = Image.new("1", (self.WIDTH, self.HEIGHT))
        self.draw = ImageDraw.Draw(self.image)
        
        self.oled.fill(0)
        self.oled.show()