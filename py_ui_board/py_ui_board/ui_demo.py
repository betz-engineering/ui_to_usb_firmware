"""
Demo app to to test the ui_board OLED, encoder, buttons and LEDs. Connect via ui_to_usb.
Revisions Rev: - and Rev: 1 are supported.
"""
import argparse
from py_ui_board.gui_primitives import GuiPrimitives, load_img
from py_ui_board.ft232h import BUTTONS
from PIL import Image, ImageFont, ImageDraw
import numpy as np
from time import sleep
from pathlib import Path

script_path = Path(__file__).resolve()

F_CREEP = ImageFont.truetype(script_path.parent / "creep.bdf", 16)
F_BIG = ImageFont.load_default(24)
colors = ["Bk", "Rd", "Gn", "Ye", "Bu", "Vt", "Tq", "Wh"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rev", choices=["-", "1"], default="1", help="Revision of the ui_to_usb board. default: 1")
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

    encoder_value = 0
    frame = 0
    is_inverted = False

    print("\nRunning demo, CTRL+C to quit ...")
    while True:
        btns, e = ui.get_inputs()

        # Handle button presses
        if btns:
            print(repr(btns))

        if btns & BUTTONS.ENC_SHORT:
            e = -encoder_value

        if btns & BUTTONS.BACK_LONG:
            print("Inverting screen.")
            is_inverted = not is_inverted
            ui.set_inverted(is_inverted)

        if e != 0 or frame == 0:
            encoder_value += e

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
            g.rectangle(128 + encoder_value, 30 + encoder_value, w=20, h=4, r=2, fill=15)

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
            dr.text((128, 62), f"Hello {encoder_value} World", fill=255, anchor="ms", font=F_BIG)
            dr.text((165, 12), "Smaller text\nlooks like\nthis!!!", fill=255, anchor="ls", font=F_CREEP)

            ui.send_img(img)
        sleep(30e-3)
        frame += 1


if __name__ == "__main__":
    main()
