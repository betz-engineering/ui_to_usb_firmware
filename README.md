# Planning for ui_to_usb

## USB interface
provided by tiny-usb. There is an example project [here](https://github.com/dragonlock2/miscboards/tree/main/wch/rvice_adc).

It will use the following endpoints

  1. EP 0 (Control): Send commands and return results
    * CMD_RESET = 0x10,
    * CMD_VERSION = 0x11,
    * CMD_IO_LEDS = 0x20,
    * CMD_IO_AUX_OE = 0x21,
    * CMD_IO_AUX_OL = 0x22,
    * CMD_OLED_FLUSH = 0x30,
    * CMD_OLED_BRIGHTNESS = 0x30,
    * CMD_OLED_INVERTED = 0x31,
  2. EP 0x81 (IN): Interrupt with guaranteed timeslot every 1ms.
    * For encoder and button data towards the PC
  3. EP 0x01 (OUT): Bulk. Uses the remaining bandwidth. For framebuffer updates.
     A complete framebuffer always needs to be written in one go. It has 8192 bytes.
     After sending, there needs to be a 4 ms quiet period before sending the next FB.
