#pragma once

#include <stdbool.h>
#include <stdint.h>

// defines the state of the 2 CS_N pins depending on which chip is selected
typedef enum { SELECT_OLED = 2, SELECT_MCP = 1, SELECT_NONE = 3 } t_ui_cs_n;

// The functions below need to be implemented in a board-specific .c file.

// Initiate a blocking 8 bit SPI transaction. Transmit val on SDO, MSB first.
// return received data from SDI.
// Used for sending configuration and reading the MCP23
// Don't touch the CS_N pin.
uint8_t ui_spi_rx_tx(uint8_t val);

// Set the state of the 2 CS_N pins: bit1: CS_N_MCP, bit0: CS_N_OLED
void ui_set_cs_n(t_ui_cs_n val);

// Set the state of the D_C pin (1 = command, 0 = data for the SSD1322)
void ui_set_dc(bool val);

// Get the state of the MCP23Sxx INT pin
bool ui_get_int(void);

// Return a running number of elapsed milliseconds / system ticks / clock cycles
// used to distinguish between short and long push
unsigned ui_get_cycles(void);

// Min. duration for a long-press event. In [cycles], as defined above.
extern const unsigned ui_t_long_press;

// --------------------------------
//  Optional Interrupt functions
// --------------------------------
// Initiate a non-blocking N bit SPI transmission.
// Used to send 1 row of framebuffer data to the OLED.
// Ideally this is implemented with DMA or interrupts if possible.
// Don't touch the CS_N pins.
// By default, a blocking transfer using ui_spi_rx_tx() will be used
void ui_spi_tx_chunk(uint8_t *buf, unsigned len);

// Return true as long as the above SPI transmission is in progress.
bool ui_spi_is_busy(void);

// Enable / disable the interrupt for mcp23_isr(). Only needed if interrupts are actually used.
void ui_set_int_enabled(bool val);

// --------------------------------
//  Usage Scenarios
// --------------------------------
// The different scenarios show 3 ways how the library can be used.
// The goal is to read out the MCP23 port-expander with the lowest possible delay
// to catch every transition on the 2 quadrature signals from the encoder.
// Lost transitions lead to missing encoder steps, making the UI feel glitchy.
//
// # Scenario 1: Purely polling based. Blocking SPI. No interrupts.
// main loop calls ui_board_poll() with > 500 Hz, which calls mcp23_isr() internally.
// The 3 interrupt functions above are not implemented by the user.
// It must be guaranteed that there are no major delays between invocations to ui_board_poll().
// SPI transfers are purely blocking. ui_board_poll() sends a maximum of 1 row of framebuffer data
// at a time, to keep its blocking time short.
// This scenario does not permit blocking the main loop for > 2 ms or so, where 0.2 ms are
// already used up for the transmission of the pixel data.
//
// # Scenario 2: Polling and background SPI transmission
// The microcontroller can transfer a chunk of data to the SPI peripheral in the background,
// using an interrupt or DMA based driver.
// ui_board_poll() still needs to be called by a fast running main-loop as above, because
// mcp23_isr() is called by ui_board_poll() only.
// When a new frame is sent to the OLED, ui_board_poll() calls ui_spi_tx_chunk() to setup
// a background transfer of one row worth of pixel data. While the data is sent, control
// is handed back to the main loop.
// In the next iteration, ui_spi_is_busy() is called to check if the transfer is
// finished and if the next row can be sent.
// ui_set_int_enabled() is not implemented by the user.
// In this scenario, little CPU time is wasted during the transfer of data to the OLED display.
// However, the main loop still can't be blocked for > 2 ms.
//
// # Scenario 3: Interrupt based MCP23 reading (recommended)
// The MCP23 has a physical interrupt pin which goes high when any of its input pins
// change. This is used to trigger a rising edge interrupt for calling mcp23_isr().
// This is ideal for slow running main loops with long-blocking operations (like
// drawing a complicated GUI).
// The main loop still needs to call ui_board_poll() every 100 ms or so, just to detect
// button long-press events.
// The user needs to setup the interrupt for calling mcp23_isr() and also implement
// ui_set_int_enabled() to enable / disable this interrupt.
// Note that interrupt based reading can be used together with Scenario 1 or 2 above.
// In Scenario 2, make sure to call mcp23_isr() once when the background SPI transmission
// is completed.
