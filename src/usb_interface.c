#include "ch32v20x.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_spi.h"
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
    uint8_t button_flags;  // Bit 0: Btn1, Bit 1: Btn2
    int8_t encoder_delta;  // Relative delta since last send
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
            ui_init();
            // ACK the transfer (no data stage needed)
            return tud_control_status(rhport, request);

        case CMD_VERSION:
            // Reply with data
            return tud_control_xfer(rhport, request, (void *)fw_version, strlen(fw_version));

        case CMD_IO_LEDS:
            set_leds(request->wValue);
            return tud_control_status(rhport, request);

        case CMD_BTNS_ENC:
            packet.button_flags = get_button_flags();
            packet.encoder_delta = get_encoder_ticks(true);
            activity_count++;
            return tud_control_xfer(rhport, request, (void *)&packet, sizeof(packet));

        case CMD_OLED_BRIGHTNESS:
            set_brightness(MIN(request->wValue, 16));
            return tud_control_status(rhport, request);

        case CMD_OLED_INVERTED:
            set_inverted(request->wValue);
            return tud_control_status(rhport, request);

        default:
            // Unknown RPC -> STALL (Python will raise USBError)
            return false;
        }
    }

    return true;
}
