# ui_to_usb Rev: -
A user-space python library to work with the older version of ui_to_usb (Rev: -) based on the FTDI 232H chip.

This library works but has a serious flaw: the encoder has to be polled at very high polling rates (around 2 kHz), which will
eat around 50 % of the CPU cycles of a modern CPU core.
