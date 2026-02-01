# Firmware for ui_to_usb Rev: 1

[ui_to_usb](https://betz-engineering.ch/open_hardware/ui_to_usb/) is a USB Interface Add-On for [ui_board_1u](https://betz-engineering.ch/open_hardware/ui_board_1u/).

As of now there are 2 hardware revisions of ui_to_usb:

  1. __Rev: -__: based on a FTDI FH282, which doesn't have programmable firmware and has performance issues polling the rotary encoder.
  2. __Rev: 1__: based on a CH32V203 USB capable microcontroller. This is the firmware package for this board.

## USB interface
The USB support is based on the excellent TinyUSB library. It implements the following endpoints:

  1. EP 0 (Control): Send commands and return results
     * 0x10: CMD_RESET, re-initialize the ui_board
     * 0x11: CMD_VERSION, return firmware version string
     * 0x20: CMD_IO_LEDS, set state of both LEDs
     * 0x21: CMD_IO_AUX_OE, set output enable of pins on AUX-IO connector
     * 0x22: CMD_IO_AUX_OL, set output level of pins on AUX-IO connector
     * 0x30: CMD_OLED_FLUSH, flush ongoing bulk transfers and start receiving a new framebuffer
     * 0x30: CMD_OLED_BRIGHTNESS, set OLED brightness level from 0 (off) to 16 (max)
     * 0x31: CMD_OLED_INVERTED, set OLED inverted shades on or off. Can be used to minimize burn in.
  2. EP 0x81 (IN): Interrupt with guaranteed timeslot every 10ms.
     * uint8: button status bit field, int8: encoder steps since last call
  3. EP 0x01 (OUT): Bulk. For framebuffer updates.
     A complete framebuffer always needs to be written in one go. It has 8192 bytes.
     After sending, there needs to be a 4 ms quiet period before sending the next FB.

On the host PC side, these endpoints can be easily accessed with libusb / pylibusb.
In the future I will explore if more native kernel drivers could be used (mouse-wheel events, keyboard LEDs, etc.).

## Compilation
This project requires [platform.io](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html#installation-installer-script). Once installed, compile the firmware with

```bash
pio run
```

To put the ui_to_usb in bootloader mode, keep the `BOOT0` button pushed and trigger a reset. The it should be possible to load the firmware with

```bash
pio run -t upload
```
