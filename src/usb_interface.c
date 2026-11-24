#include "ch32v20x.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_spi.h"
#include "i2c_lib.h"
#include "main.h"
#include "ssd1322.h"
#include "tusb.h"
#include "ui_board.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

// Struct for Device -> Host (Input)
typedef struct __attribute__((packed)) {
    uint32_t event_flags;   // Bit 0: Btn1, Bit 1: Btn2
    int32_t encoder_value;  // Absolute encoder value since boot
} input_packet_t;

#define FRAME_SIZE 8192

// If no new USB frames are received for this amount of time, start a fresh
// frame Short enough to fit in the inter-frame gap, long enough to cover USB
// jitter
#define SYNC_TIMEOUT_MS 4

static int n_written = 0;
static unsigned last_packet_time = 0;
static unsigned activity_count = 0;  // blink the Status LED on activity

void vendor_task(void) {
    if (!tud_vendor_mounted())
        return;

    // -----------------------------------------------------------
    //  Bulk endpoint to receive framebuffer data from the PC
    // -----------------------------------------------------------
    // Check if there is data available in the USB buffer
    if (tud_vendor_available()) {
        unsigned now = millis();

        // ---------------------------------
        //  Synchronization Logic
        // ---------------------------------
        // If the bus has been silent for > 4ms, assume this is a NEW frame.
        if ((now - last_packet_time) > SYNC_TIMEOUT_MS) {
            n_written = 0;  // Reset pointer to start of framebuffer
        }
        // Update timestamp
        last_packet_time = now;

        // ---------------------------------
        //  Read from USB
        // ---------------------------------
        // Read whatever is available (up to packet size)
        uint8_t buffer[64];
        int n_rx = tud_vendor_read(buffer, sizeof(buffer));
        if (n_rx <= 0)
            return;

        // ---------------------------------
        //  Write to display buffer
        // ---------------------------------
        // Only write if we haven't overflown the frame
        if (n_written < FRAME_SIZE) {
            // Don't write more then one framebuffer
            int n_remaining = FRAME_SIZE - n_written;
            if (n_rx > n_remaining)
                n_rx = n_remaining;
            send_fb(n_written == 0, n_rx, buffer);

            n_written += n_rx;

            // Comment the 2 lines below to _require_ a quiet period longer than
            // SYNC_TIMEOUT_MS before being ready to accept the next frame.
            if (n_written >= FRAME_SIZE) {
                n_written = 0;
                activity_count++;
            }
        }
    }
    GPIO_WriteBit(GPIOA, PIN_LED, !(activity_count & (1 << 4)));
}

// Command IDs
enum {
    CMD_RESET = 0x10,
    CMD_VERSION = 0x11,
    CMD_BTNS_ENC = 0x20,
    CMD_IO_LEDS = 0x21,
    // CMD_IO_AUX_OE = 0x28,
    // CMD_IO_AUX_OL = 0x29,
    // CMD_OLED_FLUSH = 0x30,
    CMD_OLED_BRIGHTNESS = 0x31,
    CMD_OLED_INVERTED = 0x32,
    CMD_I2C_CONFIG = 0x40,
    CMD_I2C_TRANSACTION = 0x41,
};

#ifndef GIT_REV
#define GIT_REV "?"
#endif

const static char fw_version[] = GIT_REV;

// ---------------------------------
//  Control endpoint for commands
// ---------------------------------
// Return true to ACK, false to STALL (error)
bool tud_vendor_control_xfer_cb(uint8_t rhport,
                                uint8_t stage,
                                tusb_control_request_t const *request) {
    input_packet_t packet = {0};

    // 1. SETUP STAGE (Host sends the command)
    if (stage == CONTROL_STAGE_SETUP) {
        switch (request->bRequest) {
        case CMD_RESET:
            reset_ui();
            ui_init(request->wValue);  // 0: UI_BOARD, 1: UI_BOARD_1U
            // ACK the transfer (no data stage needed)
            return tud_control_status(rhport, request);

        case CMD_VERSION:
            // Reply with data
            return tud_control_xfer(rhport, request, (void *)fw_version, strlen(fw_version));

        case CMD_IO_LEDS:
            set_leda(request->wValue & 0xF);
            set_ledb((request->wValue >> 4) & 0xF);
            return tud_control_status(rhport, request);

        case CMD_BTNS_ENC:
            packet.event_flags = get_event_flags();
            packet.encoder_value = get_encoder_ticks(request->wValue & 1);
            activity_count++;
            return tud_control_xfer(rhport, request, (void *)&packet, sizeof(packet));

        case CMD_OLED_BRIGHTNESS:
            set_brightness(MIN(request->wValue, 16));
            return tud_control_status(rhport, request);

        case CMD_OLED_INVERTED:
            set_inverted(request->wValue);
            return tud_control_status(rhport, request);

        case CMD_I2C_CONFIG:
            // TODO STALL on invalid parameters
            // wValue has the pin numbers (4 bit each)
            // wIndex has the SCL frequency in kHz
            i2c_gpio_init(request->wValue & 0xF, (request->wValue >> 4) & 0xF);
            i2c_timer_init(((unsigned)request->wIndex) * 1000);
            return tud_control_status(rhport, request);

            // case CMD_I2C_TRANSACTION:
            // TODO check and store wLength
            //     i2c_start_xfer(uint8_t addr, const uint8_t *tx, size_t tx_len, uint8_t *rx,
            //     size_t rx_len);

        default:
            // Unknown RPC -> STALL (Python will raise USBError)
            return false;
        }
    }

    return true;
}
