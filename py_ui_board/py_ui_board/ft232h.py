"""User-space library for the ui_to_usb Rev: -  based on FT232H
Note that the FT232H has major problems polling the encoder fast enough, especially
while the framebuffer is written. Try to use Rev: 1 of the hardware if possible.
"""
from enum import IntEnum, IntFlag
import threading
from pyftdi.spi import SpiController, SpiGpioPort, SpiPort
from PIL import Image
import time
import numpy as np


class B_IO(IntFlag):
    # MCP23 pinout
    ENC_A = 1 << 1
    ENC_B = 1 << 2
    ENC_SW = 1 << 3
    LEDB_R = 1 << 4
    LEDB_G = 1 << 5
    LEDB_B = 1 << 6
    BACK_SW = 1 << 7
    LEDA_R = 1 << 8
    LEDA_G = 1 << 9
    LEDA_B = 1 << 10
    # Not connected
    NC = (1 << 0) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15)


class R_MCP(IntEnum):
    # Registers of MCP23 IO expander. We always use 16 bit SPI transactions
    # As SPI is MSB first, we have to start with the _B register address
    IODIR = 0x01
    IPOL = 0x03
    GPINTEN = 0x05
    DEFVAL = 0x07
    INTCON = 0x09
    IOCON = 0x0B
    GPPU = 0x0D
    INTF = 0x0F
    INTCAP = 0x11
    GPIO = 0x13
    OLAT = 0x15


class B_IOCON(IntFlag):
    # Bit definition of the MCP23 IOCON register
    BANK = 1 << 7  # Separate register addresses into 2 banks
    MIRROR = 1 << 6  # OR the 2 interrupts from bank A and B together
    SEQOP = 1 << 5  # Disable sequential operation (no auto increment)
    DISSLW = 1 << 4  # Slew rate limit disabled on SDA
    HAEN = 1 << 3  # Enable hardware address strapping pins
    ODR = 1 << 2  # Open drain interrupt pin
    INTPOL = 1 << 1  # active high interrupt pin


class Mcp23:
    OPCODE_W = 0x40
    OPCODE_R = 0x41
    T_LONG = 0.5  # Min. duration of a long button push [s]

    # 4 bit lookup table for Gray-code transitions
    GRAY_TABLE = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)

    def __init__(self, p: SpiPort, gpio: SpiGpioPort):
        self.p = p
        self.gpio = gpio

        # Internal state for decoding encoder / buttons
        self.io_state = 0
        self.enc = 0
        self.last_ticks = 0
        self.enc_sum = 0
        self.ts_back_sw = 0.0
        self.ts_enc_sw = 0.0
        self.button_flags = 0
        self.t_is_running = False

        # A special mode (Byte mode with IOCON.BANK = 0) causes the address pointer
        # to toggle between associated A/B register pairs.
        self.mcp_write(R_MCP.IOCON, int(B_IOCON.INTPOL | B_IOCON.DISSLW | B_IOCON.SEQOP | B_IOCON.MIRROR), 1)
        # Disable LEDS
        self.o_val = int(B_IO.LEDA_R | B_IO.LEDA_G | B_IO.LEDA_B | B_IO.LEDB_R | B_IO.LEDB_G | B_IO.LEDB_B)
        self.mcp_write(R_MCP.OLAT, self.o_val)
        # GPIO direction
        self.mcp_write(R_MCP.IODIR, int(B_IO.ENC_A | B_IO.ENC_B | B_IO.ENC_SW | B_IO.BACK_SW | B_IO.NC))

    def mcp_write(self, addr: int, val: int | bytes, txlen=2):
        if isinstance(val, int):
            val = val.to_bytes(txlen)
        self.gpio.write(0x90)  # CS_IO = 0
        self.p.write(bytes([Mcp23.OPCODE_W, addr]) + val, start=False, stop=False)
        self.gpio.write(0xB0)  # CS_IO = 1

    def mcp_read16(self, addr: int):
        self.gpio.write(0x90)  # CS_IO = 0
        dat = self.p.exchange(bytes([Mcp23.OPCODE_R, addr]), readlen=2, start=False, stop=False)
        self.gpio.write(0xB0)  # CS_IO = 1
        return dat

    def set_led(self, leda=None, ledb=None):
        """Set the state of the LEDs from 0 - 7. The bits correspond to [R, G, B]"""
        if leda is not None:
            self.o_val |= B_IO.LEDA_R | B_IO.LEDA_G | B_IO.LEDA_B
            if leda & 1:
                self.o_val &= ~B_IO.LEDA_R
            if leda & 2:
                self.o_val &= ~B_IO.LEDA_G
            if leda & 4:
                self.o_val &= ~B_IO.LEDA_B

        if ledb is not None:
            self.o_val |= B_IO.LEDB_R | B_IO.LEDB_G | B_IO.LEDB_B

            if ledb & 1:
                self.o_val &= ~B_IO.LEDB_R

            if ledb & 2:
                self.o_val &= ~B_IO.LEDB_G

            if ledb & 4:
                self.o_val &= ~B_IO.LEDB_B

            self.mcp_write(R_MCP.OLAT, self.o_val)

    def poll(self, i=0):
        """Decodes encoder positions. Run this in a tight loop > 100 Hz"""
        io_state = int.from_bytes(self.mcp_read16(R_MCP.GPIO))

        if io_state == self.io_state:
            return

        ts = time.monotonic()

        # Buttons
        rising = (~self.io_state) & io_state
        falling = self.io_state & (~io_state)

        # On button push, latch the current cycle count
        if falling & B_IO.ENC_SW:
            self.ts_enc_sw = ts
        if falling & B_IO.BACK_SW:
            self.ts_back_sw = ts

        # Set instantaneous button state in [1, 0]
        self.button_flags &= ~0xF
        if not (io_state & B_IO.ENC_SW):
            self.button_flags |= 1 << 0
        if not (io_state & B_IO.BACK_SW):
            self.button_flags |= 1 << 1

        # On release, check if it was a long [5, 4] or a short press [3, 2]
        # and set the bits in button_flags accordingly
        if i > 0:
            if rising & B_IO.ENC_SW:
                if (ts - self.ts_enc_sw) > Mcp23.T_LONG:
                    self.button_flags |= 1 << 4
                else:
                    self.button_flags |= 1 << 2

            if rising & B_IO.BACK_SW:
                if (ts - self.ts_back_sw) > Mcp23.T_LONG:
                    self.button_flags |= 1 << 5
                else:
                    self.button_flags |= 1 << 3

        # Rotary encoder
        # read the current encoder state into enc {B, A}
        enc = 0
        if io_state & B_IO.ENC_B:
            enc = 1
        if io_state & B_IO.ENC_A:
            enc |= 2

        # Decode current and previous encoder state with a 4 bit lookup table, accumulate steps
        # The encoder makes 4 electrical steps / detent.
        # so the actual useful value is enc_sum >> 2. The LSB may jitter due to switch bouncing.
        if i > 0:
            self.enc_sum += Mcp23.GRAY_TABLE[(self.enc << 2) | enc]

        self.enc = enc
        self.io_state = io_state

    def start_poll_thread(self, t_delay=5e-3):
        if self.t_is_running:
            raise RuntimeError("thread already running")

        self.t_is_running = True

        def loop():
            i = 0
            while self.t_is_running:
                self.poll(i)
                i += 1
                time.sleep(t_delay)

        self.t = threading.Thread(target=loop)
        self.t.start()

    def stop_poll_thread(self):
        if not self.t_is_running:
            return

        self.t_is_running = False
        self.t.join()

    def get_encoder_ticks(self, reset=False):
        """returns number of encoder ticks (and direction)
        if reset is false, returns accumulated encoder ticks
        if reset is true, returns difference to last call
        """
        tmp = self.enc_sum
        ret = tmp - self.last_ticks

        if reset:
            self.last_ticks = tmp

        return ret >> 2

    def get_button_flags(self):
        """returns flags indicating button event
        instantaneous state (in the 2 LSBs) and short and long press
        events of the encoder and back-button:
        {back_long_press, enc_long_press, back_short_press, enc_short_press, back, enc},
        the press-events get cleared automatically on returning from this function."""
        ret = self.button_flags
        self.button_flags &= 0x3
        return ret

    def get_inputs(self):
        """return state of user inputs since last call: button_flags, encoder_delta
        Call this for every frame of the GUI main-loop
        button_flags indicates button events since last call:
            {BTN1_LONG, BTN0_LONG, BTN1_SHORT, BTN0_SHORT, BTN1_STATE, BTN0_STATE}
        encoder_delta is the number of ticks since last call (sign indicates direction)
        """
        return self.get_button_flags(), self.get_encoder_ticks(True)

    def get_gpios(self):
        """Get raw MCP23 GPIO pin state"""
        return self.io_state


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

        self.reset()

    def reset(self):
        try:
            self.mcp.stop_poll_thread()
        except Exception:
            pass

        # Reset the chips
        self._gpio.write(0x30)
        time.sleep(10e-3)

        # Release the reset
        self._gpio.write(0xB0)
        time.sleep(10e-3)

        self.ssd1322 = Ssd1322(self._spi_p, self._gpio)
        self.mcp = Mcp23(self._spi_p, self._gpio)

        # expose the public methods
        self.set_led = self.mcp.set_led
        self.get_inputs = self.mcp.get_inputs
        self.set_inverted = self.ssd1322.set_inverted
        self.set_brightness = self.ssd1322.set_brightness
        self.send_img = self.ssd1322.send_img

        self.mcp.start_poll_thread()
