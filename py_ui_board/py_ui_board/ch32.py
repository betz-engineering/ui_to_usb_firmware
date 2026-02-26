"""User-space library for the ui_to_usb Rev: 1  based on Ch32V203"""
from .ft232h import BUTTONS
from enum import IntEnum, IntFlag
from PIL import Image
import numpy as np
import struct
import usb.core
import usb.util


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


class UiBoard:
    def __init__(self, dev: usb.core.Device | None = None):
        if dev is None:
            dev = find_ui_board_devices()[0]
        self.dev = dev
        dev.set_configuration()
        self.led_state = 0  # {0, LEDB_B, LEDB_G, LEDB_R, 0, LEDA_B, LEDA_G, LEDA_R,}

    def reset(self):
        self.dev.ctrl_transfer(REQ.H2D, CMD.RESET, 0, 0)

    def set_led(self, leda=None, ledb=None):
        """set the LED color. Value is from 0 - 7"""
        if leda is not None:
            self.led_state &= ~7
            self.led_state |= leda & 7

        if ledb is not None:
            self.led_state &= ~(7 << 4)
            self.led_state |= (ledb & 7) << 4

        self.dev.ctrl_transfer(REQ.H2D, CMD.IO_LEDS, self.led_state, 0)

    def set_inverted(self, val=False):
        """invert the display (prevents burn-in if done periodically)"""
        self.dev.ctrl_transfer(REQ.H2D, CMD.OLED_INVERTED, val, 0)

    def set_brightness(self, val=16):
        """set OLED brightness (0 = off, 1 - 16 = on)"""
        self.dev.ctrl_transfer(REQ.H2D, CMD.OLED_BRIGHTNESS, val, 0)

    def get_fw_version(self):
        """return firmware version string (output from git describe)"""
        data = self.dev.ctrl_transfer(REQ.D2H, CMD.VERSION, 0, 0, 64)
        return data.tobytes().decode("utf-8")

    def get_inputs(self):
        """return state of user inputs since last call: button_flags, encoder_delta
        Call this for every frame of the GUI main-loop
        button_flags indicates button events since last call as BUTTONS enum.
        encoder_delta is the number of ticks since last call (sign indicates direction)
        """
        data = self.dev.ctrl_transfer(REQ.D2H, CMD.BTNS_ENC, 0, 0, 2)
        button_flags, encoder_delta = struct.unpack("Bb", data)
        return BUTTONS(button_flags), int(encoder_delta)

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
