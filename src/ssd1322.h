#pragma once
#include <stdbool.h>
#include <stdint.h>

void init_ssd1322(void);

// set OLED brightness (0 = off, 1 - 16 = on)
void set_brightness(uint8_t val);

// invert the display
void set_inverted(bool val);

// Send a blob of framebuffer data (USB packet by packet)
void send_fb(bool prefix_cmd, unsigned count, uint8_t *buf);