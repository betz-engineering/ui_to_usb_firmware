# Firmware for ui_to_usb Rev: 1

[ui_to_usb](https://betz-engineering.ch/open_hardware/ui_to_usb/) is a USB Interface Add-On for ui_board and [ui_board_1u](https://betz-engineering.ch/open_hardware/ui_board_1u/).

The board is supported by the [py_ui_board](https://github.com/betz-engineering/py_ui_board) GUI library.
To use it directly from python, only the libusb based driver is needed: [ui_to_usb.py](https://github.com/betz-engineering/py_ui_board/blob/main/src/py_ui_board/drivers/ui_to_usb.py).

As of now there are 2 hardware revisions of ui_to_usb:

  1. __Rev: -__: based on a FTDI FT232H, which doesn't have programmable firmware and makes the host PC suffer from performance issues when polling the rotary encoder at high speeds. There is an [experimental python library](py_ui_board/py_ui_board/ft232h.py), which is obsolete at this point.
  2. __Rev: 1__: based on a CH32V203 USB capable microcontroller. This repo contains the firmware and user-space [library](py_ui_board/py_ui_board/ch32.py) for this board.

## USB interface
The USB support is based on the excellent TinyUSB library.

Most functionality is achieved through the control endpoint (EP0), as defined in [usb_interface.c](src/usb_interface.c):

  * 0x10: CMD_RESET, re-initialize the ui_board. Argument is the board variant (0: ui_board, 1: ui_board_1u)
  * 0x11: CMD_VERSION, return firmware version string
  * 0x20: CMD_BTNS_ENC, get the events since last call and absolute encoder ticks since boot as int32. Set argument to 1 to reset the encoder value.
  * 0x21: CMD_IO_LEDS, set state of both LEDs
  * 0x28: CMD_IO_AUX_OE, set output enable of pins on AUX-IO connector (not implemented yet)
  * 0x29: CMD_IO_AUX_OL, set output level of pins on AUX-IO connector (not implemented yet)
  * 0x31: CMD_OLED_BRIGHTNESS, set OLED brightness level from 0 (off) to 16 (max)
  * 0x32: CMD_OLED_INVERTED, set OLED inverted mode on or off. Can be used to minimize burn-in.

A bulk endpoint (EP1) is used for framebuffer updates. A complete framebuffer always needs to be written in one go. It has 8192 bytes.
After sending, there needs to be a 4 ms quiet period before sending the next FB.

On the host PC side, these endpoints can be easily accessed with libusb / pyusb.

In the future I will explore if more native kernel drivers could be used (mouse-wheel events, keyboard LEDs, etc.).

## Controlling the extension port
A 12 pin extension port is available on the PCB. It connects directly to the GPIOs of the CH32. PORTB0 - 7 (except PORTB2) and PORTA13 - 15 are available.

The GPIO-control functions for PORTB have been made directly accessible to the USB interface. Pins can be configured as input, push-pull output or open-drain output.

In addition, the firmware implements a fully featured I2C controller. SCL and SDA can be configured for any of the PORTB pins.

## Compilation
This project requires [platform.io](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html#installation-installer-script). Once installed, compile the firmware with

```bash
pio run
```

To put the ui_to_usb in bootloader mode, keep the `BOOT0` button pushed and trigger a reset. The it should be possible to load the firmware with

```bash
pio run -t upload
```

## USB permissions
Add the following UDEV rule to allow read and write access to ui_to_usb:

```bash
sudo vim /etc/udev/rules.d/50-libusb-shared.rules

  SUBSYSTEM=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="05dc", MODE="0666", TAG+="uaccess"

sudo udevadm control --reload-rules
sudo udevadm trigger
```

ui_to_usb uses a [free shared VID:PID pair](https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt) provided by the v-usb project.
