"""
Demo app to to test the ui_board OLED, encoder, buttons and LEDs. Connect via ui_to_usb.
Revisions Rev: - and Rev: 1 are supported.
"""

import argparse
import threading
from collections import Counter
from pathlib import Path
from time import monotonic, sleep

import usb.core
from PIL import Image, ImageDraw, ImageFont
from py_ui_board.ch32 import BOARD_TYPE, EV, I2C_RESULT
from py_ui_board.gui_primitives import GuiPrimitives

script_path = Path(__file__).resolve()

F_CREEP = ImageFont.truetype(script_path.parent / "creep.bdf", 16)
F_BIG = ImageFont.load_default(24)
colors = ["Bk", "Rd", "Gn", "Ye", "Bu", "Vt", "Tq", "Wh"]

FS3000_ADDR = 0x28


class I2cHammer(threading.Thread):
    """Stress test: read the FS3000 air velocity sensor as fast as possible in the background.
    Its 5 byte reply (checksum, data_h, data_l, 2 generic bytes) sums to 0 mod 256,
    so corrupted reads are detected, not only failed transfers."""

    def __init__(self, ui):
        super().__init__(daemon=True)
        self.ui = ui
        self.n = 0  # transactions
        self.errors = Counter()  # I2C_RESULT / "checksum" / "usb" -> count
        self.raw = 0  # last valid 12 bit sensor value
        self.running = True

    def run(self):
        while self.running:
            try:
                res, dat = self.ui.i2c_transfer(FS3000_ADDR, b"", 5)
            except usb.core.USBError as e:
                self.errors[f"usb: {e}"] += 1
                continue
            finally:
                self.n += 1
            if res != I2C_RESULT.OK:
                self.errors[res.name] += 1
            elif sum(dat[:5]) & 0xFF:
                self.errors["checksum"] += 1
                print("checksum error:", dat[:5].hex(" "))
            else:
                self.raw = ((dat[1] & 0x0F) << 8) | dat[2]

    def n_errors(self):
        return sum(self.errors.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rev",
        choices=["-", "1"],
        default="1",
        help="Revision of the ui_to_usb board. default: 1",
    )
    parser.add_argument(
        "--i2c",
        action="store_true",
        help="stress test: hammer a FS3000 sensor on PB0 = SDA, PB1 = SCL while redrawing every frame",
    )
    args = parser.parse_args()

    if args.rev == "1":
        from py_ui_board.ch32 import UiBoard, find_ui_board_devices

        devs = find_ui_board_devices()
        if len(devs) == 0:
            print("No ui_to_usb Rev: 1 boards found!")
            exit(-1)
        print("Found devices:", [dev.serial_number for dev in devs])
        print("Connecting to first one.")
    else:
        from py_ui_board.ft232h import UiBoard

    ui = UiBoard()
    ui.reset()

    if args.rev == "1":
        print("Firmware version:", ui.get_fw_version())

    frame = 0
    is_inverted = False
    is_ui_1u = True

    hammer = None
    if args.i2c:
        ui.i2c_config(400, sda_pin=0, scl_pin=1)
        hammer = I2cHammer(ui)
        hammer.start()
        t_stats, n_stats = monotonic(), 0

    print("\nRunning demo, CTRL+C to quit ...")
    reset = False
    while True:
        events, encoder_value = ui.get_inputs(reset)
        reset = EV.ENC_S in events

        # Handle button presses
        if events:
            print(repr(events))

        if events & EV.ENC_L:
            is_ui_1u = not is_ui_1u
            if is_ui_1u:
                print("Initializing for ui_board_1u")
                ui.reset(BOARD_TYPE.UI_BOARD_1U)
            else:
                print("Initializing for ui_board")
                ui.reset(BOARD_TYPE.UI_BOARD)

        if events & EV.BACK_L:
            print("Inverting screen.")
            is_inverted = not is_inverted
            ui.set_inverted(is_inverted)

        if hammer is not None and monotonic() - t_stats >= 1:
            dt, t_stats = monotonic() - t_stats, monotonic()
            rate, n_stats = (hammer.n - n_stats) / dt, hammer.n
            print(
                f"I2C: {rate:6.0f} / s, {hammer.n} total, raw = {hammer.raw:4d}, errors: {dict(hammer.errors)}"
            )

        if events != 0 or frame == 0 or hammer is not None:
            img = Image.new("L", (256, 64), color=0)

            dr = ImageDraw.Draw(img)

            # Anti-aliased drawing
            g = GuiPrimitives(img)

            # Bar with Greyscale levels
            for i in range(16):
                g.rectangle(i * 16 + 8 + encoder_value, 32, 16, 64, fill=15 - i)

            # Creepy smiley
            g.dot(128, 22, 40, outline=5, linewidth=4, fill=0)
            g.dot(120, 15, 10, outline=15)
            g.dot(136, 15, 10, outline=15)
            g.rectangle(
                128 + encoder_value, 30 + encoder_value, w=20, h=4, r=2, fill=15
            )

            # Dot-bar indicating which screen is on
            g.dot_bar(encoder_value + 3, r=3, N=6)

            # LED status
            leda_val = encoder_value & 0x7
            ledb_val = (encoder_value >> 3) & 0x7
            ui.set_led(leda=leda_val, ledb=ledb_val)
            g.dot(11, 22, 20, outline=0xF, linewidth=1, fill=0)
            g.dot(44, 22, 20, outline=0xF, linewidth=1, fill=0)
            g.draw.flush()

            dr.text((11, 22), colors[leda_val], fill=255, anchor="mm", font=F_CREEP)
            dr.text((44, 22), colors[ledb_val], fill=255, anchor="mm", font=F_CREEP)

            # Text / icons
            dr.text(
                (128, 62),
                f"Hello {encoder_value} World",
                fill=255,
                anchor="ms",
                font=F_BIG,
            )
            if hammer is None:
                txt = "Smaller text\nlooks like\nthis!!!"
            else:
                txt = f"n {hammer.n}\nerr {hammer.n_errors()}\nraw {hammer.raw}"
            dr.text(
                (165, 12),
                txt,
                fill=255,
                anchor="ls",
                font=F_CREEP,
            )

            ui.send_img(img)
        sleep(30e-3)
        frame += 1


if __name__ == "__main__":
    main()
