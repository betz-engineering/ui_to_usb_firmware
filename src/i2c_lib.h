#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    I2C_OK,           // Transaction succeeded
    I2C_ERR_NACK,     // address or data byte was not acknowledged
    I2C_ERR_BUS,      // SDA low before START (bus busy / stuck)
    I2C_ERR_TIMEOUT,  // SCL held low longer than 25 ms
    I2C_BUSY = 15,    // Transaction in progress
} i2c_result_t;

// Setup I2C pins to use (PORTB0 - 7 only). Returns true on success
bool i2c_gpio_init(unsigned scl_pin_id, unsigned sda_pin_id);

// Setup I2C clock-rate in [Hz]. Returns true on success
// clamped to: f_scl_min = 550 Hz, f_scl_max = 400 kHz (use ~385 kHz for Fast-mode timing)
bool i2c_timer_init(unsigned f_scl);

// Write tx_len bytes, then (repeated START) read rx_len bytes. Both may be 0
// (tx_len = rx_len = 0 is an address-only probe / SMBus quick command).
// addr is 7 bit wide (right shifted)
// Returns false if a transfer is still in progress.
bool i2c_start_xfer(uint8_t addr, const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len);

// see i2c_result_t for meaning of the return values
i2c_result_t i2c_get_result(void);

