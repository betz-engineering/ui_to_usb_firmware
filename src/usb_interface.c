#include "ch32v20x.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_spi.h"
#include "main.h"
#include "ssd1322.h"
#include "tusb.h"
#include "ui_board.h"
#include <stdint.h>
#include <string.h>

// Struct for Device -> Host (Input)
typedef struct __attribute__((packed)) {
    uint8_t buttons;     // Bit 0: Btn1, Bit 1: Btn2
    int8_t encoder_val;  // Relative delta since last send
} input_packet_t;

#define FRAME_SIZE 8192

// If no new USB frames are received for this amount of time, start a fresh frame
// Short enough to fit in the inter-frame gap, long enough to cover USB jitter
#define SYNC_TIMEOUT_MS 4

static unsigned byte_index = 0;
static unsigned last_packet_time = 0;

void vendor_task(void) {
    if (!tud_vendor_mounted())
        return;

    // -----------------------------------------------------------
    //  Interrupt endpoint to send btn. and encoder state to PC
    // -----------------------------------------------------------
    // The host polls us every 10ms.
    // Check if the previous Interrupt packet has been sent
    if (tud_vendor_write_available() >= sizeof(input_packet_t)) {
        input_packet_t packet;
        packet.buttons = get_button_flags();
        packet.encoder_val = get_encoder_ticks(true);
        tud_vendor_write(&packet, sizeof(packet));
    }

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
            byte_index = 0;  // Reset pointer to start of framebuffer
        }
        // Update timestamp
        last_packet_time = now;

        // ---------------------------------
        //  Read from USB
        // ---------------------------------
        // Read whatever is available (up to packet size)
        uint8_t buffer[64];
        unsigned count = tud_vendor_read(buffer, sizeof(buffer));
        if (count <= 0)
            return;

        // ---------------------------------
        //  Write to display buffer
        // ---------------------------------
        // Only write if we haven't overflown the frame

        if (byte_index < FRAME_SIZE) {
            // Calculate how much we can actually write
            unsigned remaining = FRAME_SIZE - byte_index;
            if (count > remaining)
                count = remaining;
            send_fb(byte_index == 0, count, buffer);
            byte_index += count;
        }

        // Note that once the frame is written completely, we implicitly require a
        // a quiet period > SYNC_TIMEOUT_MS before we are ready to accept the next
        // frame and before the byte_index is reset.
    }
}

// Command IDs
enum {
    CMD_RESET = 0x10,
    CMD_VERSION = 0x11,
    CMD_IO_LEDS = 0x20,
    // CMD_IO_AUX_OE = 0x28,
    // CMD_IO_AUX_OL = 0x29,
    // CMD_OLED_FLUSH = 0x30,
    CMD_OLED_BRIGHTNESS = 0x30,
    CMD_OLED_INVERTED = 0x31,
};

static char fw_version[] = "v1.0.4-beta";

// ---------------------------------
//  Control endpoint for commands
// ---------------------------------
// Return true to ACK, false to STALL (error)
bool tud_vendor_control_xfer_cb(uint8_t rhport,
                                uint8_t stage,
                                tusb_control_request_t const *request) {
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

        case CMD_OLED_BRIGHTNESS:
            set_brightness(request->wValue);
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
