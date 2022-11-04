import board
import time
import math
import busio
import displayio
import terminalio
from adafruit_display_text import label
import adafruit_displayio_ssd1306
from microcontroller import Pin

class Display:
    def __init__(self, sda:Pin, scl:Pin,
                 width:int, height:int, border:int,
                 invert:bool=False) -> None:
        """ setup OLED display

        Args:
            sda (Pin): SDA pin
            scl (Pin): SCL pin
            width (int): width of display in px
            height (int): height of display in px
            border (int): border around outside of display
            invert (bool, optional): display colors inverted. Defaults to False.
        """
        # free up all pins that may have previously been used for displays
        displayio.release_displays()

        # Use for I2C
        i2c = busio.I2C(scl=SCL, sda=SDA); display_bus = displayio.I2CDisplay(i2c, device_address=0x3C)

        # setup display object
        self.WIDTH = width; self.HEIGHT = height; self.BORDER = border
        self.display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=self.WIDTH, height=self.HEIGHT)

        self.setup_canvas()

    def setup_canvas(self):
        # Make the display context
        self.splash = displayio.Group()
        self.display.show(self.splash)

        color_bitmap = displayio.Bitmap(self.WIDTH, self.HEIGHT, 1)
        color_palette = displayio.Palette(1)
        color_palette[0] = 0xFFFFFF  # White

        bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=color_palette, x=0, y=0)
        self.splash.append(bg_sprite)

        self.clear_canvas()
    
    def clear_canvas(self):
        # Draw a smaller inner rectangle
        inner_bitmap = displayio.Bitmap(self.WIDTH - self.BORDER * 2, self.HEIGHT - self.BORDER * 2, 1)
        inner_palette = displayio.Palette(1)
        inner_palette[0] = 0x000000  # Black
        inner_sprite = displayio.TileGrid(
            inner_bitmap, pixel_shader=inner_palette, x=self.BORDER, y=self.BORDER
        )
        self.splash.append(inner_sprite)

    def text(self, text):
        self.clear_canvas()
        # create text object
        text_area = label.Label(
            terminalio.FONT, text=text, color=0xFFFFFF, y=self.HEIGHT // 2 - 1
        )

        # place text in x center of screen
        text_width = text_area.width
        text_area.x = math.floor(self.WIDTH/2 - text_width/2)
        self.splash.append(text_area)


SDA = board.GP26
SCL = board.GP27

disp = Display(SDA, SCL, width=128, height=32, border=0)
disp.text("its nerf or nothin")

while True:
    pass
