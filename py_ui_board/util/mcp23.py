import time
import threading
from enum import IntEnum, IntFlag
from pyftdi.spi import SpiPort, SpiGpioPort


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
        self.enc_acc = 0
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

    def set_leda(self, rgb_value: int):
        """Set the state of the left LED (D1) from 0 - 7
        the 3 LSBs correspond to the state of the [R, G, B] LEDs"""
        self.o_val |= B_IO.LEDA_R | B_IO.LEDA_G | B_IO.LEDA_B

        if rgb_value & 1:
            self.o_val &= ~B_IO.LEDA_R

        if rgb_value & 2:
            self.o_val &= ~B_IO.LEDA_G

        if rgb_value & 4:
            self.o_val &= ~B_IO.LEDA_B

        self.mcp_write(R_MCP.OLAT, self.o_val)

    def set_ledb(self, rgb_value: int):
        """Set the state of the right LED (D2) from 0 - 7
        the 3 LSBs correspond to the state of the [R, G, B] LEDs"""
        self.o_val |= B_IO.LEDB_R | B_IO.LEDB_G | B_IO.LEDB_B

        if rgb_value & 1:
            self.o_val &= ~B_IO.LEDB_R

        if rgb_value & 2:
            self.o_val &= ~B_IO.LEDB_G

        if rgb_value & 4:
            self.o_val &= ~B_IO.LEDB_B

        self.mcp_write(R_MCP.OLAT, self.o_val)

    def poll(self):
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
            self.button_flags |= 1
        if not (io_state & B_IO.BACK_SW):
            self.button_flags |= 2

        # On release, check if it was a long [9, 8] or a short press [5, 4]
        # and set the bits in button_flags accordingly
        if rising & B_IO.ENC_SW:
            if (ts - self.ts_enc_sw) > Mcp23.T_LONG:
                self.button_flags |= 1 << 8
            else:
                self.button_flags |= 1 << 4

        if rising & B_IO.BACK_SW:
            if (ts - self.ts_back_sw) > Mcp23.T_LONG:
                self.button_flags |= 2 << 8
            else:
                self.button_flags |= 2 << 4

        # Rotary encoder
        # read the current encoder state into enc {B, A}
        enc = 0
        if io_state & B_IO.ENC_B:
            enc = 1
        if io_state & B_IO.ENC_A:
            enc |= 2

        # Decode current and previous encoder state with a 4 bit lookup table, accumulate steps
        self.enc_acc += Mcp23.GRAY_TABLE[(self.enc << 2) | enc]

        # The encoder makes 4 electrical steps / detent.
        if enc != 0:
            if self.enc_acc >= 2:
                self.enc_sum += 1

            if self.enc_acc <= -2:
                self.enc_sum -= 1

            self.enc_acc = 0

        self.enc = enc
        self.io_state = io_state

    def start_poll_thread(self, t_delay=2e-3):
        if self.t_is_running:
            raise RuntimeError("thread already running")

        self.t_is_running = True

        def loop():
            while self.t_is_running:
                self.poll()
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
        if reset:
            self.enc_sum = 0
        return tmp

    def get_button_flags(self):
        """returns flags indicating button event
        instantaneous state (in the 2 LSBs) and short and long press
        events of the encoder and back-button:
        {back_long_press, enc_long_press, 0, 0, back_short_press, enc_short_press, 0, 0, back, enc},
        the press-events get cleared automatically on returning from this function."""
        return self.button_flags

    def get_gpios(self):
        """Get raw MCP23 GPIO pin state"""
        return self.io_state
