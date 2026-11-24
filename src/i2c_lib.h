#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Call i2c_timer_isr() from a periodic timer interrupt.
// Tick period = 1 / (4 * f_SCL), e.g. 2.5 us for 100 kHz.
// Keep ISR latency jitter below ~40 us if SMBus devices are on the bus (tHIGH max = 50 us).
#ifndef I2C_TIMEOUT_TICKS
#define I2C_TIMEOUT_TICKS 10000  // max. clock stretching: 25 ms at 2.5 us / tick (SMBus timeout)
#endif

enum i2c_err {
    I2C_OK = 0,
    I2C_ERR_NACK = 1,     // address or data byte was not acknowledged
    I2C_ERR_BUS = 2,      // SDA low before START (bus busy / stuck)
    I2C_ERR_TIMEOUT = 3,  // SCL held low longer than I2C_TIMEOUT_TICKS
};

void i2c_gpio_init(unsigned scl_pin_id, unsigned sda_pin_id);

// Write tx_len bytes, then (repeated START) read rx_len bytes. Both may be 0
// (tx_len = rx_len = 0 is an address-only probe / SMBus quick command).
// Returns false if a transfer is still in progress.
bool i2c_start_xfer(uint8_t addr, const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len);

// -1 = busy, otherwise an enum i2c_err value of the last transfer
int i2c_result(void);

void i2c_timer_isr(void);
