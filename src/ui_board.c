#include "ui_board.h"
#include "ssd1322.h"
#include <generated/csr.h>
#include <stdint.h>
#include <stdio.h>

// Min. duration for a long press in [clock-cycles]
#define T_LONG 50000000

// Bit 1, 2 and 3 of MCP23_OPCODE_W encode the hardware address (strapping pins)
#define MCP23_OPCODE_W 0x40
#define MCP23_OPCODE_R (MCP23_OPCODE_W | 1)

// Registers of MCP23 IO expander. We always use 16 bit SPI transactions
// As SPI is MSB first, we have to start with the _B register address
#define MCP23_IODIR 0x01
#define MCP23_IPOL 0x03
#define MCP23_GPINTEN 0x05
#define MCP23_DEFVAL 0x07
#define MCP23_INTCON 0x09
#define MCP23_IOCON 0x0B
#define MCP23_GPPU 0x0D
#define MCP23_INTF 0x0F
#define MCP23_INTCAP 0x11
#define MCP23_GPIO 0x13
#define MCP23_OLAT 0x15

// Bits of MCP23_IOCON (configuration register)
#define BANK (1 << 7)    // Separate register addresses into 2 banks
#define MIRROR (1 << 6)  // OR the 2 interrupts from bank A and B together
#define SEQOP (1 << 5)   // Disable sequential operation (no auto increment)
#define DISSLW (1 << 4)  // Slew rate limit disabled on SDA
#define HAEN (1 << 3)    // Enable hardware address strapping pins
#define ODR (1 << 2)     // Open drain interrupt pin
#define INTPOL (1 << 1)  // active high interrupt pin

// encoder and button state. Need to disable the interrupt before writing these
static volatile int enc_sum = 0;
static volatile uint16_t gpio_state = 0;
static unsigned button_flags = 0;
static unsigned output_value = 0;

// Write a 16 bit register-pair (suffix _A and _B)
static void mcp23_write16(uint8_t addr, uint16_t val) {
    // value will be sent MSB-first
    ui_phy_mosi_write((MCP23_OPCODE_W << 24) | (addr << 16) | val);
    ui_phy_control_write((32 << 8) | 1);
    while (!(ui_phy_status_read() & 1))
        ;
}

// Write a 8 bit register
static void mcp23_write8(uint8_t addr, uint8_t val) {
    // value will be sent MSB-first
    ui_phy_mosi_write((MCP23_OPCODE_W << 24) | (addr << 16) | (val << 8));
    ui_phy_control_write((24 << 8) | 1);
    while (!(ui_phy_status_read() & 1))
        ;
}

// Read a 16 bit register-pair (suffix _A and _B)
static uint16_t mcp23_read16(uint8_t addr) {
    // value will be sent MSB-first
    ui_phy_mosi_write((MCP23_OPCODE_R << 24) | (addr << 16));
    ui_phy_control_write((32 << 8) | 1);
    while (!(ui_phy_status_read() & 1))
        ;
    return ui_phy_miso_read();
}

// 4 bit lookup table for Gray-code transitions
static const int8_t enc_table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void ui_isr(void) {
    static unsigned cycle_enc_sw = 0, cycle_back_sw = 0;

    // Read clock cycle counter
    unsigned cycles = ui_cycles_read();

    // A SPI write transaction may be already in progress (ssd1322 send_fb() for example)
    // Wait for this transaction to complete
    while (!(ui_phy_status_read() & 1))
        ;

    // Save SPI peripheral state
    unsigned tmp_cs = ui_phy_cs_read();
    // Read the MCP23 IO pin state
    ui_phy_cs_write(MCP23_CS);
    const unsigned val = mcp23_read16(MCP23_GPIO);
    // Restore SPI peripheral state
    ui_phy_cs_write(tmp_cs);

    // Print GPIO state
    // for (unsigned i=0; i<=15; i++)
    //     printf("%d", (val >> (15 - i)) & 1);
    // printf("\n");

    // Print encoder state
    // printf("%d%d\n", (val & IO_ENC_B) > 0, (val & IO_ENC_A) > 0);

    // decode buttons states (rising or falling edge of the input signal)
    unsigned rising = (~gpio_state) & val;
    unsigned falling = gpio_state & (~val);

    // On button push, latch the current cycle count
    if (falling & IO_ENC_SW)
        cycle_enc_sw = cycles;
    if (falling & IO_BACK_SW)
        cycle_back_sw = cycles;

    // Set instantaneous button state in [1, 0]
    button_flags &= ~0x3;
    if (!(val & IO_ENC_SW))
        button_flags |= 1;
    if (!(val & IO_BACK_SW))
        button_flags |= 2;

    // On release, check if it was a long [9, 8] or a short press [5, 4]
    // and set the bits in button_flags accordingly
    if (rising & IO_ENC_SW) {
        if ((cycles - cycle_enc_sw) > T_LONG)
            button_flags |= 1 << 4;
        else
            button_flags |= 1 << 2;
    }
    if (rising & IO_BACK_SW) {
        if ((cycles - cycle_back_sw) > T_LONG)
            button_flags |= 2 << 4;
        else
            button_flags |= 2 << 2;
    }

    // # Rotary encoder
    static int8_t enc_acc = 0, enc_d = 0;
    int8_t enc = 0;
    // read the current encoder state into enc {B, A}
    if (val & IO_ENC_B)
        enc = 1;
    if (val & IO_ENC_A)
        enc |= 2;
    // Decode current and previous encoder state with a 4 bit lookup table, accumulate steps
    enc_acc += enc_table[(enc_d << 2) | enc];

    // The encoder makes 4 electrical steps / detent.
    // Mechanically stable states (between detents) are 0b10, 0b11, 0b01
    // 0b00 is always a unstable state (on a detent)
    // Only check the accumulator when in a stable state
    if (enc != 0) {
        if (enc_acc >= 2)
            enc_sum++;
        if (enc_acc <= -2)
            enc_sum--;
        enc_acc = 0;
    }
    enc_d = enc;
    gpio_state = val;
}

void ui_init(void) {
    ui_ev_enable_write(0);

    // # Power ON reset
    ui_reset_n_write(0);
    busy_wait(1);
    ui_reset_n_write(1);
    busy_wait(1);

    // # Init MCP23
    ui_phy_clk_divider_write(CONFIG_CLOCK_FREQUENCY /
                             10000000);  // Set SPI clock to 10 MHz (MCP23 maximum)
    ui_phy_cs_write(MCP23_CS);
    // A special mode (Byte mode with IOCON.BANK = 0) causes the address pointer
    // to toggle between associated A/B register pairs.
    mcp23_write8(MCP23_IOCON, INTPOL | DISSLW | SEQOP | MIRROR);
    // GPIO direction
    mcp23_write16(MCP23_IODIR, IO_ENC_A | IO_ENC_B | IO_ENC_SW | IO_BACK_SW | IO_NC);
    // enable interrupts for switches and encoder
    mcp23_write16(MCP23_GPINTEN, IO_ENC_A | IO_ENC_B | IO_ENC_SW | IO_BACK_SW);
    // interrupt on any input change
    mcp23_write16(MCP23_INTCON, 0);

    set_leda(0);
    set_ledb(0);

    // # Init the OLED
    init_ssd1322();

    ui_ev_enable_write(1);
}

int get_encoder_ticks(bool reset) {
    ui_ev_enable_write(0);
    int ret = enc_sum;
    if (reset)
        enc_sum = 0;
    ui_ev_enable_write(1);
    return ret;
}

unsigned get_button_flags(void) {
    ui_ev_enable_write(0);
    unsigned ret = button_flags;
    button_flags &= 0x3;  // clear the button-push event flags
    ui_ev_enable_write(1);
    return ret;
}

uint16_t get_gpios(void) { return gpio_state; }

void set_leds(unsigned rgb_value) {
    output_value |= IO_LEDA_R | IO_LEDA_G | IO_LEDA_B | IO_LEDB_R | IO_LEDB_G | IO_LEDB_B;

    if (rgb_value & 1)
        output_value &= ~IO_LEDA_R;

    if (rgb_value & 2)
        output_value &= ~IO_LEDA_G;

    if (rgb_value & 4)
        output_value &= ~IO_LEDA_B;

    if (rgb_value & 0x10)
        output_value &= ~IO_LEDB_R;

    if (rgb_value & 0x20)
        output_value &= ~IO_LEDB_G;

    if (rgb_value & 0x40)
        output_value &= ~IO_LEDB_B;

    ui_phy_cs_write(MCP23_CS);
    mcp23_write16(MCP23_OLAT, output_value);
}

void set_leda(unsigned rgb_value) {
    output_value |= IO_LEDA_R | IO_LEDA_G | IO_LEDA_B;

    if (rgb_value & 1)
        output_value &= ~IO_LEDA_R;

    if (rgb_value & 2)
        output_value &= ~IO_LEDA_G;

    if (rgb_value & 4)
        output_value &= ~IO_LEDA_B;

    ui_phy_cs_write(MCP23_CS);
    mcp23_write16(MCP23_OLAT, output_value);
}

void set_ledb(unsigned rgb_value) {
    output_value |= IO_LEDB_R | IO_LEDB_G | IO_LEDB_B;

    if (rgb_value & 1)
        output_value &= ~IO_LEDB_R;

    if (rgb_value & 2)
        output_value &= ~IO_LEDB_G;

    if (rgb_value & 4)
        output_value &= ~IO_LEDB_B;

    ui_phy_cs_write(MCP23_CS);
    mcp23_write16(MCP23_OLAT, output_value);
}
