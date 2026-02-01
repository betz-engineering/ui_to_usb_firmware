#pragma once
#include <stdbool.h>
#include <stdint.h>

// Button flags indicating events
#define EV_ENC_S (1 << 4)
#define EV_ENC_L (1 << 8)
#define EV_BACK_S (1 << 5)
#define EV_BACK_L (1 << 9)

// MCP23 pin assignment
#define IO_NC ((1 << 0) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15))
#define IO_ENC_A (1 << 1)
#define IO_ENC_B (1 << 2)
#define IO_ENC_SW (1 << 3)
#define IO_LEDB_R (1 << 4)
#define IO_LEDB_G (1 << 5)
#define IO_LEDB_B (1 << 6)
#define IO_BACK_SW (1 << 7)
#define IO_LEDA_R (1 << 8)
#define IO_LEDA_G (1 << 9)
#define IO_LEDA_B (1 << 10)

// Mapping of chip-select signals
#define SSD1322_CS (1 << 0)
#define MCP23_CS (1 << 1)

// # Call this once to initialize
void ui_init(void);

// Call in main loop
void ui_board_poll(void);

// # Call these whenever

// if reset is true, returns number of encoder ticks (and direction) since last call
// if reset is false, returns accumulated encoder ticks
int get_encoder_ticks(bool reset);

// returns flags indicating instantaneous state (in the 2 LSBs) and short and long press
// events of the encoder and back-button:
// {back_long_press, enc_long_press, back_short_press, enc_short_press, back, enc},
// the press-events get cleared automatically on returning from this function.
unsigned get_button_flags(void);

// # Set the LED status, bits of rgb_value are {B, G, R}
void set_leda(unsigned rgb_value);
void set_ledb(unsigned rgb_value);
void set_leds(unsigned rgb_value);

// Get raw MCP23 GPIO input values
uint16_t get_gpios(void);
