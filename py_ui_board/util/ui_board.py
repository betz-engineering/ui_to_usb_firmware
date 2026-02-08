from pyftdi.spi import SpiController
from aggdraw import Brush, Pen, Draw
from PIL import Image
import time
from ui_board.mcp23 import Mcp23
from ui_board.ssd1322 import Ssd1322


class UiBoard:
    def __init__(self, url="ftdi://ftdi:232h/1", freq=10e6):
        # Somewhat annoyingly, pyftdi expects that all CS signals are
        # next to each other. Unfortunately that's not how I designed the PCB
        # so we have to manually control the CS_IO pin through the GPIO class
        spi = SpiController()
        spi.configure(url, cs_count=1)
        self._spi_p = spi.get_port(cs=0, freq=freq, mode=0)

        # Get GPIO port to manage extra pins
        self._gpio = spi.get_gpio()
        # 0x80 is reset, 0x20 is CS_IO, 0x10 is DC
        self._gpio.set_direction(0xB0, 0xB0)

        # Reset the chips
        self._gpio.write(0x30)
        time.sleep(10e-3)

        # Release the reset
        self._gpio.write(0xB0)
        time.sleep(10e-3)

        self.ssd1322 = Ssd1322(self._spi_p, self._gpio)
        self.mcp = Mcp23(self._spi_p, self._gpio)

        # expose the public methods
        self.set_leda = self.mcp.set_leda
        self.set_ledb = self.mcp.set_ledb

        self.set_inverted = self.ssd1322.set_inverted
        self.set_brightness = self.ssd1322.set_brightness
        self.send_img = self.ssd1322.send_img


# ----------------------------------
#  Various graphical helpers
# ----------------------------------
def c(v=0):
    """scale 4 bit color to 8 bit"""
    return v * 17


def load_img(name, scale=1.0):
    i_lbl = Image.open(name)
    dims = (round(i_lbl.width * scale), round(i_lbl.height * scale))
    return i_lbl.convert("L").resize(dims, Image.Resampling.LANCZOS)


class GuiPrimitives:
    def __init__(self, img: Image.Image):
        self.draw = Draw(img)
        self.W = 256
        self.H = 64

    def _bh(self, fill=None, outline=None, linewidth=1.0):
        """brush and pen helper, to set fill and outline properties"""
        b = None
        p = None
        if fill is not None:
            b = Brush(c(fill))
        if outline is not None:
            p = Pen(c(outline), linewidth)
        return b, p

    def dot(self, x, y, d=3, **kwargs):
        """a circle centered at x and y with diameter d"""
        self.draw.ellipse((x - d / 2, y - d / 2, x + d / 2, y + d / 2), *self._bh(**kwargs))

    def dot_bar(self, active=1, N=3, r=3):
        """A column of N dots on the right side of the screen. One dot is active."""
        dist = 3 * r
        y = (self.H - (N - 1) * dist) / 2
        for i in range(N):
            self.dot(self.W - r, y, 2 * r, fill=15 if i == active else 2)
            y += dist

    def rectangle(self, x, y, w=30, h=15, r=0, **kwargs):
        """a rectangle centered at x and y with width w and height h and corner radius r"""
        self.draw.rounded_rectangle((x - w / 2, y - h / 2, x + w / 2, y + h / 2), r, *self._bh(**kwargs))
