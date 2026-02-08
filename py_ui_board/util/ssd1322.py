from pyftdi.spi import SpiPort, SpiGpioPort
from PIL import Image
import numpy as np


class Ssd1322:
    # Initialization for NHD-2.8-25664UCB2 OLED display
    # negative = command, positive = data
    # fmt: off
    INIT_VALS = [
        -0xFD, 0x12,     # Unlock OLED driver IC
        -0xAE,           # Display OFF (blank)
        -0x15, 0x1C, 0x5B,  # Set column address to 1C, 5B
        -0x75, 0x00, 0x3F,  # Set row address to 00, 3F
        -0xB3, 0x91,     # set clock to 80 fps
        -0xCA, 0x3F,     # Multiplex ratio, 1/64, 64 COMS enabled
        -0xA2, 0x00,     # Set offset, the display map starting line is COM0
        -0xA1, 0x00,     # Set start line position
        -0xA0, 0x14, 0x11,   # Set remap, horiz address increment, disable colum address remap,
                         #  enable nibble remap, scan from com[N-1] to COM0, disable COM split odd even
        -0xB5, 0x00,     # Disable GPIO inputs
        -0xAB, 0x01,     # Select external VDD
        # Display enhancement A, 0xA0: external VSL, 0xA2: internal VSL, 0xB5: normal, 0xFD: enhanced low GS
        -0xB4, 0xA0, 0xB5,
        -0xC1, 0x7F,     # Contrast current, 256 steps, default is 0x7F
        -0xC7, 0x0B,     # Master contrast current (brightness), 16 steps, default is 0x0F
        -0xB9,           # load linear gamma table
        -0xB1, 0xF4,     # Reset period / first pre-charge period Length
        -0xD1, 0xa2, 0x20,  # Display enhancement B
        -0xBB, 0x17,     # Pre-charge voltage
        -0xB6, 0x08,     # Second pre-charge period = 8 clks
        -0xBE, 0x04,     # VCOMH: Set Common Pins Deselect Voltage Level as 0.8 * VCC
        -0xA6,           # Normal display
        -0xA9,           # Disable partial display mode
        -0xAF            # Display ON
    ]
    # fmt: on

    def __init__(self, spi_p: SpiPort, gpio: SpiGpioPort):
        self._spi_p = spi_p
        self._gpio = gpio
        self.last_cmd = None
        self.send_init()
        self.set_gamma()

    def _D_C(self, v=0):
        if v:
            self._gpio.write(0xB0)
        else:
            self._gpio.write(0xA0)

    def send_cmd(self, cmds=[], datas=[]):
        self._D_C(0)
        self._spi_p.write(bytes(cmds), stop=False)  # write commands
        self._D_C(1)
        self._spi_p.write(bytes(datas), start=False)  # write datas
        self.last_cmd = cmds[-1]

    def send_init(self):
        last = len(Ssd1322.INIT_VALS) - 1
        for i, v in enumerate(Ssd1322.INIT_VALS):
            self._D_C(v >= 0)
            self._spi_p.write(bytes([abs(v)]), start=(i == 0), stop=(i == last))

    def set_inverted(self, v=False):
        """Invert the display shades.
        Useful to toggle this every now and then avoid burn-in."""
        self.send_cmd([0xA7] if v else [0xA6])

    def set_brightness(self, val=16):
        """Set overall brightness of OLED from 0 - 16
        0 means the OLED will power down"""
        if val < 0:
            val = 0
        if val > 16:
            val = 16
        if val == 0:
            self.send_cmd([0xAE])  # display off
        else:
            # display on, set brightness (0 - 15)
            self.send_cmd([0xAF, 0xC7], [val - 1])

    def set_gamma(self, max_value=180, vals=None):
        # Load custom gamma table
        if vals is None:
            vals = [round(i * i / 196 * max_value) for i in range(15)]
        self.send_cmd([0xB8], vals)
        self.send_cmd([0x00])

    def send_fb(self, buf: bytes):
        # Only send the command if we really have to.
        if self.last_cmd != 0x5C:
            self.send_cmd([0x5C])
        self._spi_p.write(buf)

    def send_img(self, img: Image.Image):
        """send a PIL Image to the display"""
        arr = np.array(img, dtype=np.uint8)
        arr4 = arr >> 4  # 8-bit → 4-bit
        # Pack two pixels per byte
        packed = (arr4[:, ::2] << 4) | arr4[:, 1::2]
        self.send_fb(packed.flatten().tobytes())
