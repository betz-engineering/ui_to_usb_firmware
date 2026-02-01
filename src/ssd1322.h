#pragma once
#include <stdbool.h>
#include <stdint.h>

void init_ssd1322(void);

// set OLED brightness (0 = off, 1 - 16 = on)
void set_brightness(uint8_t val);

// invert the display
void set_inverted(bool val);

void send_fb(bool prefix_cmd, unsigned count, uint8_t *buf);

// send a certain rectangular window of the framebuffer to the display
// void send_window_4(unsigned x1, unsigned y1, unsigned x2, unsigned y2, uint8_t *data);
