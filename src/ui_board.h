#pragma once
#include <stdbool.h>
#include <stdint.h>

// This library supports both:
//   * legacy ui_board, dual color LED, no back button, using 8-bit MCP23S08
//   * ui_board_1u, 2x RGB LED, back button, using 16 bit MCP23S17
// Select the flavor at runtime when calling ui_init();
typedef enum { UI_BOARD, UI_BOARD_1U, UI_BOARD_1U_291V } t_ui_board_type;

// Meaning of the bits in the value returned by get_event_flags()
// It indicates which events happened since the last call
// Instantaneous value (1 = pushed)
#define EV_ENC (1 << 0)   // encoder button
#define EV_BACK (1 << 1)  // back button
// On push
#define EV_ENC_P (1 << 4)
#define EV_BACK_P (1 << 5)
// Short push and release
#define EV_ENC_S (1 << 8)
#define EV_BACK_S (1 << 9)
// Long push
#define EV_ENC_L (1 << 12)
#define EV_BACK_L (1 << 13)
// Encoder ticks. Alternative to get_encoder_ticks()
#define EV_ROT_CCW (1 << 16)
#define EV_ROT_CW (1 << 17)

// Call this once to initialize the ui_board
// before calling this:
//   * Initialize the SPI peripheral (10 MHz max.)
//   * Set the RESET pin low for 1 ms, then high and wait another 1 ms
void ui_init(t_ui_board_type value);

// Call this in the main loop. If interrupts are used: > 10 Hz, otherwise > 500 Hz
// for each call, it:
//   * reads the inputs (including the encoder quadrature signals)
//   * sends pixel data to the OLED (one row per call)
//   * updates the LEDs (once per frame)
// returns true when every row of the framebuffer has been sent to the OLED.
// To avoid glitches, only draw to the framebuffer after it returns true.
// Set new_frame to false to avoid starting the next framebuffer transmission.
// Useful to keep the SPI bus quiet if no pixels have been modified.
bool ui_board_poll(bool new_frame);

// Optional: call this on the rising edge of the MCP23Sxx INT pin. It will read the inputs only.
// When using background transfers for ui_spi_tx_chunk(), also call this when the
// ui_spi_tx_chunk() transfer is finished.
void mcp23_isr(void);

// if reset is true, returns number of encoder ticks (and direction) since last call
// if reset is false, returns accumulated encoder ticks
int get_encoder_ticks(bool reset);

// returns the instantaneous state of the encoder and back button (in the 2 LSBs)
// the other bits are used to indicate events. See the EV_ flags above.
unsigned get_event_flags(void);

// # Set the LED status, bits of rgb_value are {B, G, R}
void set_leda(unsigned rgb_value);
void set_ledb(unsigned rgb_value);

// Get raw MCP23 GPIO input values
uint16_t get_gpios(void);
