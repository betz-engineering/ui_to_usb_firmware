"""User-space library for the ui_to_usb Rev: 1  based on Ch32V203"""

import struct
from enum import IntEnum, IntFlag
from time import sleep

import numpy as np
import usb.core
import usb.util
from PIL import Image


class EV(IntFlag):
    # Meaning of the bits in the value returned by get_event_flags()
    # It indicates which events happened since the last call
    # Instantaneous value (1 = pushed)
    ENC = 1 << 0  # encoder button
    BACK = 1 << 1  # back button
    # On push
    ENC_P = 1 << 4
    BACK_P = 1 << 5
    # Short push and release
    ENC_S = 1 << 8
    BACK_S = 1 << 9
    # Long push
    ENC_L = 1 << 12
    BACK_L = 1 << 13
    # Encoder ticks. Alternative to get_encoder_ticks()
    ROT_CCW = 1 << 16
    ROT_CW = 1 << 17


class I2C_RESULT(IntFlag):
    OK = 0  # Transaction succeeded
    ERR_NACK = 1  # address or data byte was not acknowledged
    ERR_BUS = 2  # SDA low before START (bus busy / stuck)
    ERR_TIMEOUT = 3  # SCL held low longer than 25 ms
    BUSY = 15  # Transaction in progress


def find_ui_board_devices():
    """returns a list of valid ui_to_usb devices"""
    devices = usb.core.find(find_all=True, idVendor=0x16C0, idProduct=0x05DC)

    # Filter by manufacturer and product strings
    matching_devices = []
    for dev in devices:
        try:
            # Get manufacturer and product strings
            manufacturer = usb.util.get_string(dev, dev.iManufacturer)
            product = usb.util.get_string(dev, dev.iProduct)

            if manufacturer == "betz-engineering.ch" and product == "ui_to_usb":
                matching_devices.append(dev)
        except (usb.core.USBError, ValueError):
            # Skip devices we can't read strings from
            continue
    return sorted(matching_devices, key=lambda d: d.serial_number)


class REQ(IntEnum):
    # Device-to-host, Vendor-specific, Device recipient
    D2H = 0xC0
    # Host-to-device, Vendor-specific, Device recipient
    H2D = 0x40


# USB bRequest type for control transfers
class CMD(IntEnum):
    RESET = 0x10
    VERSION = 0x11
    BTNS_ENC = 0x20
    IO_LEDS = 0x21
    # IO_AUX_OE = 0x28
    # IO_AUX_OL = 0x29
    # OLED_FLUSH = 0x30
    OLED_BRIGHTNESS = 0x31
    OLED_INVERTED = 0x32
    I2C_CONFIG = 0x40
    I2C_TRANSACTION = 0x41
    I2C_RESULT = 0x42


# // This library supports both:
# //   * legacy ui_board, dual color LED, no back button, using 8-bit MCP23S08
# //   * ui_board_1u, 2x RGB LED, back button, using 16 bit MCP23S17
class BOARD_TYPE(IntEnum):
    UI_BOARD = 0
    UI_BOARD_1U = 1
    UI_BOARD_1U_291V = 2


class UiBoard:
    def __init__(self, dev: usb.core.Device | None = None):
        if dev is None:
            dev = find_ui_board_devices()[0]
        self.dev = dev
        dev.set_configuration()
        self.led_state = 0  # {0, LEDB_B, LEDB_G, LEDB_R, 0, LEDA_B, LEDA_G, LEDA_R,}

    def ctrl_transfer(self, *args, **kwargs):
        """like dev.ctrl_transfer() but with 3 attempts on USB_ERROR
        works around a race condition in TinyUSB"""
        # Under heavy load (SPI and I2C), TinyUSB occasionally fails a control request
        # with an I/O error, followed by one timed out request. Retry then.
        for i in range(3):
            try:
                return self.dev.ctrl_transfer(*args, **kwargs, timeout=100)
            except usb.core.USBError:
                if i >= 2:
                    raise

    def reset(self, board_type=BOARD_TYPE.UI_BOARD_1U):
        self.ctrl_transfer(REQ.H2D, CMD.RESET, board_type, 0)

    def set_led(self, leda=None, ledb=None):
        """set the LED color. Value is from 0 - 7"""
        if leda is not None:
            self.led_state &= ~7
            self.led_state |= leda & 7

        if ledb is not None:
            self.led_state &= ~(7 << 4)
            self.led_state |= (ledb & 7) << 4

        self.ctrl_transfer(REQ.H2D, CMD.IO_LEDS, self.led_state, 0)

    def set_inverted(self, val=False):
        """invert the display (prevents burn-in if done periodically)"""
        self.ctrl_transfer(REQ.H2D, CMD.OLED_INVERTED, val, 0)

    def set_brightness(self, val=16):
        """set OLED brightness (0 = off, 1 - 16 = on)"""
        self.ctrl_transfer(REQ.H2D, CMD.OLED_BRIGHTNESS, val, 0)

    def get_fw_version(self):
        """return firmware version string (output from git describe)"""
        data = self.ctrl_transfer(REQ.D2H, CMD.VERSION, 0, 0, 64)
        return data.tobytes().decode("utf-8")

    def get_inputs(self, reset_encoder=False):
        """return state of user inputs since last call: event_flags, encoder_value
        Call this for every frame of the GUI main-loop
        event_flags indicates button events since last call as BUTTONS enum.
        encoder_value is the number of ticks since last call (sign indicates direction)
        """
        data = self.ctrl_transfer(REQ.D2H, CMD.BTNS_ENC, reset_encoder, 0, 8)
        event_flags, encoder_value = struct.unpack("Ii", data)
        return EV(event_flags), encoder_value

    def send_fb(self, buf: bytes):
        # Send a frame-buffer to the OLED display
        if len(buf) != 8192:
            raise RuntimeError("Wrong framebuffer size. Must be 8192 bytes.")
        self.dev.write(1, buf)

    def send_img(self, img: Image.Image):
        """send a PIL Image to the display"""
        arr = np.array(img, dtype=np.uint8)
        arr4 = arr >> 4  # 8-bit → 4-bit
        # Pack two pixels per byte
        packed = (arr4[:, ::2] << 4) | arr4[:, 1::2]
        self.send_fb(packed.flatten().tobytes())

    def i2c_config(self, f_scl=100, scl_pin=0, sda_pin=1):
        """Setup the I2C mode. External pullups are needed!!!
        scl_pin and sda_pin are from 0 to 7 and choose a PORTB GPIO pin
        f_scl is the clock frequency in [kHz] from 1 to 400 kHz
        """
        if scl_pin < 0 or scl_pin > 7 or sda_pin < 0 or sda_pin > 7:
            raise RuntimeError("Invalid pins. Only 0 - 7 is allowed")
        self.ctrl_transfer(REQ.H2D, CMD.I2C_CONFIG, (sda_pin << 4) | scl_pin, f_scl)

    def i2c_transfer(self, device_addr, tx_data=b"", n_rx=0):
        """Start an I2C transaction and block until it's done.
        device_addr is the 7 bit chip address
        tx_data is the data to send (min. 0, max. 64 bytes)
        n_rx is the number of bytes to receive (min. 0, max. 63 bytes)
        Returns status, received data as bytes
        """
        self.ctrl_transfer(REQ.H2D, CMD.I2C_TRANSACTION, device_addr, n_rx, tx_data)
        result, dat = None, None
        for _ in range(32):
            result, dat = self.i2c_result()
            if result != I2C_RESULT.BUSY:
                break
            sleep(1e-3)
        return result, dat

    def i2c_result(self):
        """Fetches the result of the last transaction
        returns: I2C_RESULT, received data
        """
        arr = self.ctrl_transfer(REQ.D2H, CMD.I2C_RESULT, data_or_wLength=64)
        return I2C_RESULT(arr[0]), bytes(arr[1:])
