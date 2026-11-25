#include "i2c_lib.h"
#include "ch32v20x.h"
#include <stdbool.h>

// Bit-bang I2C master driven by a periodic timer interrupt.
// Every bit takes 4 ticks and SCL is left low between bits:
//   0: set SDA   1: release SCL   2: wait for SCL high (stretching), sample SDA   3: pull SCL low

// ---------------------------------------------------------
//  Internal state
// ---------------------------------------------------------
typedef enum { I2C_IDLE, I2C_START, I2C_BYTE, I2C_STOP } i2c_state_t;
typedef enum { K_WRITE, K_RADDR, K_RDATA } byte_kind_t;

static volatile i2c_state_t state;
static volatile i2c_result_t result = I2C_OK;
static unsigned phase;     // tick within the current START / bit / STOP
static unsigned bit_idx;   // 0..8 within the current byte
static byte_kind_t kind;   // what kind of byte is processed right now
static unsigned tx9, rx9;  // 9 bit shift registers (MSB first, ACK is bit 0)

static unsigned timeout_ticks_max = 1000;  // max. clock stretching: 25 ms (SMBus timeout)
static unsigned timeout_ticks_cnt = 0;     // clock stretching timeout counter

static unsigned addr;  // 7-bit slave address
static const uint8_t *tx_buf;
static size_t tx_len, tx_idx;
static uint8_t *rx_buf;
static size_t rx_len, rx_idx;

// ---------------------------------------------------------
//  Configurable settings
// ---------------------------------------------------------
static unsigned scl_pin_mask = (1 << 6);
static unsigned sda_pin_mask = (1 << 7);

// Release line (Open-Drain Hi-Z, pulled high by external resistor)
#define SCL_HIGH() (GPIOB->BSHR = scl_pin_mask)
#define SDA_HIGH() (GPIOB->BSHR = sda_pin_mask)

// Pull line low (N-MOS active)
#define SCL_LOW() (GPIOB->BCR = scl_pin_mask)
#define SDA_LOW() (GPIOB->BCR = sda_pin_mask)

// Read actual pin voltage level (works even while configured as Open-Drain output!)
#define SCL_READ() ((GPIOB->INDR & scl_pin_mask) != 0)
#define SDA_READ() ((GPIOB->INDR & sda_pin_mask) != 0)

// PORTB 0 to 7 is supported
bool i2c_gpio_init(unsigned scl_pin_id, unsigned sda_pin_id) {
    if (scl_pin_id > 7 || sda_pin_id > 7)
        return false;

    scl_pin_mask = (1 << scl_pin_id);
    sda_pin_mask = (1 << sda_pin_id);

    // Set them high
    GPIOB->BSHR = scl_pin_mask | sda_pin_mask;

    // Configure them as Open-Drain Output, 50 MHz (0x7 = 0b0111)
    //    (Use CFGLR for pins 0..7, CFGHR for pins 8..15)
    GPIOB->CFGLR &= ~((0xF << (scl_pin_id * 4)) | (0xF << (sda_pin_id * 4)));
    GPIOB->CFGLR |= ((0x7 << (scl_pin_id * 4)) | (0x7 << (sda_pin_id * 4)));
    return true;
}

// f_scl is the I2C clock frequency. Setup TIM2 to get 4 interrupts per period.
// clamped to: f_scl_min = 550 Hz, f_scl_max = 400 kHz
bool i2c_timer_init(unsigned f_scl) {
    if (f_scl == 0)
        return false;
    unsigned reload = SystemCoreClock / (4 * f_scl) - 1;
    if (reload < 89 || reload > 0xFFFF)
        return false;

    // Set timeout to 25 ms: n_ticks = 0.025 s * 4 * f_scl
    timeout_ticks_max = f_scl / 10;

    RCC->APB1PCENR |= RCC_TIM2EN;  // Enable clock
    TIM2->CTLR1 = 0;               // counter off until a transfer starts
    TIM2->PSC = 0;
    TIM2->ATRLR = reload;  // must fit 16 bit: f_scl >= 550 Hz at 144 MHz
    TIM2->INTFR = 0;
    TIM2->DMAINTENR = TIM_UIE;
    NVIC_SetPriority(TIM2_IRQn, 0);  // highest priority: keeps SCL timing jitter low
    NVIC_EnableIRQ(TIM2_IRQn);
    return true;
}

bool i2c_start_xfer(
    uint8_t address, const uint8_t *tx, size_t tx_length, uint8_t *rx, size_t rx_length) {
    if (state != I2C_IDLE)
        return false;  // Busy

    addr = address;
    tx_buf = tx;
    tx_len = tx_length;
    tx_idx = 0;
    rx_buf = rx;
    rx_len = rx_length;
    rx_idx = 0;
    result = I2C_OK;
    phase = 0;
    timeout_ticks_cnt = 0;
    state = I2C_START;

    TIM2->CTLR1 |= TIM_CEN;  // start ticking, stops again once IDLE
    return true;
}

i2c_result_t i2c_get_result(void) { return state == I2C_IDLE ? result : I2C_BUSY; }

// Abort: release both lines and go idle
static void finish(i2c_result_t err) {
    result = err;
    SDA_HIGH();
    SCL_HIGH();
    state = I2C_IDLE;
}

// True once SCL is really high. Times out if a slave stretches the clock too long.
static bool scl_is_high(void) {
    if (SCL_READ()) {
        timeout_ticks_cnt = 0;
        return true;
    }
    if (++timeout_ticks_cnt > timeout_ticks_max)
        finish(I2C_ERR_TIMEOUT);
    return false;
}

static void send(byte_kind_t k, uint8_t b) {
    kind = k;
    // ACK bit: released (1) so the slave can ACK, or when reading: NACK (1) on the last byte
    bool ack_bit = (k != K_RDATA) || (rx_idx + 1 >= rx_len);
    tx9 = (b << 1) | ack_bit;
    rx9 = 0;
    bit_idx = 0;
    state = I2C_BYTE;
}

// After (repeated) START: write address if there is something to write (or for an
// address-only probe), otherwise read address
static void after_start(void) {
    if (tx_idx < tx_len || rx_len == 0)
        send(K_WRITE, addr << 1);
    else
        send(K_RADDR, (addr << 1) | 1);
}

static void byte_done(void) {
    if (kind == K_RDATA)
        rx_buf[rx_idx++] = rx9 >> 1;
    else if (rx9 & 1) {  // NACK from slave
        result = I2C_ERR_NACK;
        state = I2C_STOP;
        return;
    }

    if (kind != K_WRITE)  // reading
        if (rx_idx < rx_len)
            send(K_RDATA, 0xFF);
        else
            state = I2C_STOP;
    else if (tx_idx < tx_len)
        send(K_WRITE, tx_buf[tx_idx++]);
    else if (rx_len > 0)
        state = I2C_START;  // repeated START, then read
    else
        state = I2C_STOP;
}

static void i2c_timer_isr(void) {
    switch (state) {

    case I2C_IDLE:
        TIM2->CTLR1 &= ~TIM_CEN;  // nothing to do, stop the timer
        return;

    // (repeated) START: SDA falls while SCL is high. Entered with SCL low or bus idle.
    case I2C_START:
        switch (phase) {
        case 0:
            SDA_HIGH();
            break;
        case 1:
            SCL_HIGH();
            break;
        case 2:
            if (!scl_is_high())
                return;
            if (!SDA_READ()) {  // someone holds SDA low
                finish(I2C_ERR_BUS);
                return;
            }
            break;
        case 4:
            SDA_LOW();
            break;  // START condition
        case 6:
            SCL_LOW();
            phase = 0;
            after_start();
            return;
        }
        break;

    // Shift one bit out and in. Entered with SCL low.
    case I2C_BYTE:
        switch (phase) {
        case 0:
            if (tx9 & 0x100)
                SDA_HIGH();
            else
                SDA_LOW();
            tx9 <<= 1;
            break;
        case 1:
            SCL_HIGH();
            break;
        case 2:
            if (!scl_is_high())
                return;
            rx9 = (rx9 << 1) | SDA_READ();
            break;
        case 3:
            SCL_LOW();
            phase = 0;
            if (++bit_idx == 9)
                byte_done();
            return;
        }
        break;

    // STOP: SDA rises while SCL is high. Entered with SCL low.
    case I2C_STOP:
        switch (phase) {
        case 0:
            SDA_LOW();
            break;
        case 1:
            SCL_HIGH();
            break;
        case 2:
            if (!scl_is_high())
                return;
            break;
        case 4:  // tSU;STO >= 2 ticks
            SDA_HIGH();
            state = I2C_IDLE;
            return;
        }
        break;
    }
    phase++;
}

volatile unsigned n_tim2_ints = 0;

__attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used)) void TIM2_IRQHandler(void) {
    TIM2->INTFR = ~TIM_UIF;  // clear update flag (write 0 clears)
    i2c_timer_isr();
    n_tim2_ints++;
}
