#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Driven by the TIM2 update interrupt, ticking at 4 * f_SCL while a transfer is running.
// Keep ISR latency jitter below ~40 us if SMBus devices are on the bus (tHIGH max = 50 us).
#ifndef I2C_TIMEOUT_TICKS
#define I2C_TIMEOUT_TICKS 10000  // max. clock stretching: 25 ms at 2.5 us / tick (SMBus timeout)
#endif

typedef enum {
    I2C_OK = 0,
    I2C_ERR_NACK = 1,     // address or data byte was not acknowledged
    I2C_ERR_BUS = 2,      // SDA low before START (bus busy / stuck)
    I2C_ERR_TIMEOUT = 3,  // SCL held low longer than I2C_TIMEOUT_TICKS
} i2c_err_t;

// Setup I2C pins to use (PORTB0 - 7 only)
void i2c_gpio_init(unsigned scl_pin_id, unsigned sda_pin_id);

// Setup I2C clock-rate in [Hz]
// clamped to: f_scl_min = 550 Hz, f_scl_max = 400 kHz (use ~385 kHz for Fast-mode timing)
void i2c_timer_init(unsigned f_scl);

// Write tx_len bytes, then (repeated START) read rx_len bytes. Both may be 0
// (tx_len = rx_len = 0 is an address-only probe / SMBus quick command).
// Returns false if a transfer is still in progress.
bool i2c_start_xfer(uint8_t addr, const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len);

// -1 = busy, otherwise an enum i2c_err value of the last transfer
i2c_err_t i2c_result(void);

extern volatile unsigned n_ints;
