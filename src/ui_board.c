#include "ui_board.h"
// #include "frame_buffer.h"
#include "hardware_interface.h"
#include "ssd1322.h"
#include <stdint.h>

// Bit 1, 2 and 3 of MCP23_OPCODE_W encode the hardware address (strapping pins)
#define MCP23_OPCODE_W 0x40
#define MCP23_OPCODE_R (MCP23_OPCODE_W | 1)

// MCP23 pin assignment
// PORTA
#define IO_OLED_RES_N (1 << 0)  // only on ui_board
#define IO_ENC_A (1 << 1)
#define IO_ENC_B (1 << 2)
#define IO_ENC_SW (1 << 3)
#define IO_LEDB_R (1 << 4)
#define IO_LEDB_G (1 << 5)
#define IO_LEDB_B (1 << 6)
#define IO_BACK_SW (1 << 7)
// PORTB, only on ui_board_1u
#define IO_LEDA_R (1 << 0)
#define IO_LEDA_G (1 << 1)
#define IO_LEDA_B (1 << 2)

// Registers of MCP23S08 IO expander.
// On the MCP23S27 we will enable the BANK bit and always use 8 bit SPI transactions,
// so these are valid as well. To access the upper bank, add 0x10.
#define MCP23_IODIR 0
#define MCP23_IPOL 1
#define MCP23_GPINTEN 2
#define MCP23_DEFVAL 3
#define MCP23_INTCON 4
#define MCP23_IOCON 5
#define MCP23_GPPU 6
#define MCP23_INTF 7
#define MCP23_INTCAP 8
#define MCP23_GPIO 9
#define MCP23_OLAT 10
// Valid for the MCP23S17 when the BANK bit is 0 (like after RESET)
#define MCP23_IOCON_INTERLEAVED 0xB

// Bits of MCP23_IOCON (configuration register)
#define BANK (1 << 7)    // Separate register addresses into 2 banks
#define MIRROR (1 << 6)  // OR the 2 interrupts from bank A and B together
#define SEQOP (1 << 5)   // Disable sequential operation (no auto increment)
#define DISSLW (1 << 4)  // Slew rate limit disabled on SDA
#define HAEN (1 << 3)    // Enable hardware address strapping pins
#define ODR (1 << 2)     // Open drain interrupt pin
#define INTPOL (1 << 1)  // active high interrupt pin

// encoder, raw-input and button state.
static volatile int enc_sum = 0;
static volatile uint16_t gpio_state = 0;
static unsigned event_flags = 0;

// Which hardware-flavor are we connected to?
static t_ui_board_type board_type = UI_BOARD;

// 4 bit lookup table for Gray-code transitions
static const int8_t enc_table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Optional, only needed if interrupts are used
__attribute__((weak)) void ui_set_int_enabled(bool val) { (void)val; }

// Default implementation for blocking SPI transmission of the chunk (1 row of the framebuffer)
__attribute__((weak)) void ui_spi_tx_chunk(uint8_t *buf, unsigned len) {
    while (len--)
        ui_spi_rx_tx(*buf++);
    ui_set_cs_n(SELECT_NONE);
}

// By default blocking transfers are used, so it can never be busy.
__attribute__((weak)) bool ui_spi_is_busy(void) { return false; }

// Write a 8 bit register
static void mcp23_write8(uint8_t addr, uint8_t val) {
    ui_set_cs_n(SELECT_MCP);
    ui_spi_rx_tx(MCP23_OPCODE_W);
    ui_spi_rx_tx(addr);
    ui_spi_rx_tx(val);
    ui_set_cs_n(SELECT_NONE);
}

// Read a 8 bit register
static uint8_t mcp23_read8(uint8_t addr) {
    ui_set_cs_n(SELECT_MCP);
    ui_spi_rx_tx(MCP23_OPCODE_R);
    ui_spi_rx_tx(addr);
    uint8_t tmp = ui_spi_rx_tx(0);
    ui_set_cs_n(SELECT_NONE);
    return tmp;
}

void mcp23_isr(void) {
    static unsigned push_cycles[2], last_buttons = 0;
    static bool is_initialized = false;
    static int8_t enc_d = 0;
    unsigned rising = 0, falling = 0;

    // disable MCP23 interrupt pin (important if called from main)
    ui_set_int_enabled(false);

    // A non-blocking SPI transfer is still in progress, can't do anything for now.
    // I'll rely on the user to call mcp23_isr() when the background transfer is finished.
    if (ui_spi_is_busy()) {
        // D("isr: spi_busy\n");
        return;
    }

    // Read clock cycle counter
    unsigned cycles = ui_get_cycles();

    // We can skip the SPI read if no pins have changed on the MCP23 (if called from main)
    if (ui_get_int()) {
        // Read the MCP23 IO pin state (PORTA only)
        const unsigned val = mcp23_read8(MCP23_GPIO);
        // read the current encoder state
        int8_t enc = (val >> 1) & 3;

        // extract button values
        unsigned buttons = 0;
        if ((val & IO_ENC_SW) == 0)
            buttons |= 1;
        if (board_type != UI_BOARD && (val & IO_BACK_SW) == 0)
            buttons |= 2;

        // Don't send any events in the first iteration
        if (!is_initialized) {
            gpio_state = val;
            enc_d = enc;
            last_buttons = buttons;
            is_initialized = true;
            ui_set_int_enabled(true);
            return;
        }

        // Set instantaneous button state in bits 0x00F
        event_flags &= ~0xF;
        event_flags |= buttons;

        // decode buttons states (rising or falling edges)
        rising = (~last_buttons) & buttons;
        falling = last_buttons & (~buttons);
        last_buttons = buttons;

        // Decode current and previous encoder state with a 4 bit lookup table, accumulate steps
        enc_sum -= enc_table[(enc_d << 2) | enc];

        if (board_type == UI_BOARD_1U) {
            // Force the LSBs of enc_sum to zero in a certain position
            // This keeps the mechanical detents aligned with enc_sum / 4
            if (enc == 3)
                enc_sum = enc_sum & ~3;
        }

        enc_d = enc;
        gpio_state = val;
    }

    // Long / short push logic
    for (int i = 0; i < 2; i++) {
        if (rising & (1 << i)) {
            // On button push, latch the current cycle count
            event_flags |= EV_ENC_P << i;
            push_cycles[i] = cycles;
        } else if (push_cycles[i] > 0) {
            if (falling & (1 << i)) {
                // On button release, fire a short-press event
                event_flags |= EV_ENC_S << i;
                push_cycles[i] = 0;
            } else if ((cycles - push_cycles[i]) >= ui_t_long_press) {
                // On timeout, fire a long-press event
                event_flags |= EV_ENC_L << i;
                push_cycles[i] = 0;
            }
        }
    }
    ui_set_int_enabled(true);
}

// if the highest bit is set, the value shall be written to the MCP in the next call to
// ui_board_poll()
static uint8_t led_a_value = 0, led_b_value = 0;

bool ui_board_poll(bool new_frame) {
    // Read the MCP23 inputs ...
    mcp23_isr();

    // Get exclusive access to the SPI
    // ui_set_int_enabled(false);

    // A non-blocking SPI transfer is still in progress, can't do anything for now
    // if (ui_spi_is_busy()) {
    //     // D("poll: SPI busy\n");
    //     return false;
    // }

    // Send another row, if a transfer is in progress
    // bool is_done = send_row();
    // D("send_row(): %d\n", is_done);

    // if (is_done) {
    // Write the MCP23 outputs (once per frame, if needed)
    if (led_a_value & 0x80) {
        led_a_value &= 7;
        mcp23_write8(MCP23_OLAT + 0x10, led_a_value);
    }

    if (led_b_value & 0x80) {
        led_b_value &= 7;
        // LEDB is connected to bit 4, 5, 6 of PORTA
        mcp23_write8(MCP23_OLAT, (led_b_value << 4) | IO_OLED_RES_N);
    }

    // Restart transmission of the frame-buffer. One row of pixels per iteration
    // if (new_frame)
    //     send_fb();
    // }

    // Re-enable listening to MCP23 pin-changes
    // ui_set_int_enabled(true);
    return false;
}

void ui_init(t_ui_board_type value) {
    board_type = value;
    enc_sum = 0;
    event_flags = 0;

    // Use _banked_ register access on the MCP23S17, for compatibility with MCP23S08
    mcp23_write8(MCP23_IOCON_INTERLEAVED, INTPOL | DISSLW | SEQOP | BANK);
    mcp23_write8(MCP23_IOCON_INTERLEAVED, INTPOL | DISSLW | SEQOP | BANK);
    // Write IOCON once more, in case we got an MCP23S08
    mcp23_write8(MCP23_IOCON, INTPOL | DISSLW | SEQOP | BANK);

    // GPIO direction, clear a bit to configure the corresponding GPIO as output
    // This will also reset the OLED on ui_board
    mcp23_write8(MCP23_IODIR, ~(IO_OLED_RES_N | IO_LEDB_R | IO_LEDB_G | IO_LEDB_B));
    // 3 more outputs on PORTB (MCP23S17 only)
    mcp23_write8(MCP23_IODIR + 0x10, ~(IO_LEDA_R | IO_LEDA_G | IO_LEDA_B));
    // enable interrupts for switches and encoder (PORTA only)
    mcp23_write8(MCP23_GPINTEN, IO_ENC_A | IO_ENC_B | IO_ENC_SW | IO_BACK_SW);
    // for ui_board: release the OLED reset
    mcp23_write8(MCP23_OLAT, IO_OLED_RES_N);

    // # Init the OLED
    init_ssd1322();

    set_ledb(0);
    set_leda(0);
}

static int processed_ticks = 0;

int get_encoder_ticks(bool reset) {
    static int last_ticks = 0;
    int tmp = enc_sum;

    int ret = tmp - last_ticks;

    if (reset) {
        last_ticks = tmp;
        processed_ticks = 0;
    }

    // Electrical steps / tick is 4 times higher for ui_board_1u
    if (board_type == UI_BOARD_1U)
        return ret >> 2;

    return ret;
}

// Converts the encoder number into a series of left / right events
static unsigned get_rotate_events(void) {
    unsigned flags = 0;
    int ticks = get_encoder_ticks(false);

    if (ticks < processed_ticks) {
        flags |= EV_ROT_CCW;
        processed_ticks--;
    } else if (ticks > processed_ticks) {
        flags |= EV_ROT_CW;
        processed_ticks++;
    }
    return flags;
}

unsigned get_event_flags(void) {
    unsigned ret = event_flags;
    event_flags &= 0xF;  // clear the sticky button-push event flags
    ret |= get_rotate_events();
    return ret;
}

uint16_t get_gpios(void) { return gpio_state; }

void set_leda(unsigned rgb_value) {
    // LEDs are active high for the legacy ui_board and active low for ui_board_1u
    if (board_type == UI_BOARD_1U || board_type == UI_BOARD_1U_291V)
        rgb_value = ~rgb_value;

    // LEDA is connected to the lower 3 bits of PORTB
    led_a_value = 0x80 | (rgb_value & 7);
}

void set_ledb(unsigned rgb_value) {
    if (board_type == UI_BOARD_1U || board_type == UI_BOARD_1U_291V)
        rgb_value = ~rgb_value;

    // LEDA is connected to the lower 3 bits of PORTB
    led_b_value = 0x80 | (rgb_value & 7);
}
