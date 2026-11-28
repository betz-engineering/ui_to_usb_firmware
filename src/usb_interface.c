#include "ch32v20x.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_spi.h"
#include "common/tusb_types.h"
#include "i2c_lib.h"
#include "main.h"
#include "ssd1322.h"
#include "tusb.h"
#include "ui_board.h"
#include <stdbool.h>
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

// buffers for the I2C command
static uint8_t i2c_tx_buffer[64];
static uint8_t i2c_rx_buffer[64];
static unsigned i2c_n_rx = 0;
static uint8_t i2c_address = 0;

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
    // Aux GPIO commands
    CMD_GPIOB_INIT = 0x28,
    CMD_GPIOB_READ = 0x29,
    CMD_GPIOB_WRITE = 0x2a,
    CMD_GPIOB_SETBIT = 0x2b,
    CMD_GPIOB_RESETBIT = 0x2c,
    // OLED control
    CMD_OLED_BRIGHTNESS = 0x31,
    CMD_OLED_INVERTED = 0x32,
    // I2C commands
    CMD_I2C_CONFIG = 0x40,
    CMD_I2C_TRANSACTION = 0x41,
    CMD_I2C_RESULT = 0x42,
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
    static input_packet_t packet = {0};
    bool ret = false;
    i2c_result_t result;
    static unsigned tmp = 0;
    GPIO_InitTypeDef gpio = {.GPIO_Speed = GPIO_Speed_50MHz};

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

        case CMD_GPIOB_INIT:
            // wValue: see GPIOMode_TypeDef
            // wIndex: 16 bit pin mask where wValue applies
            // don't touch the I2C pins if I2C is used
            gpio.GPIO_Pin = request->wValue;
            gpio.GPIO_Mode = request->wIndex;
            GPIO_Init(GPIOB, &gpio);
            return tud_control_status(rhport, request);

        case CMD_GPIOB_READ:
            tmp = GPIO_ReadInputData(GPIOB);
            return tud_control_xfer(rhport, request, (void *)&tmp, 2);

        case CMD_GPIOB_WRITE:
            // Careful, can't be used at the same time as I2C mode
            GPIO_Write(GPIOB, request->wValue);
            return tud_control_status(rhport, request);

        case CMD_GPIOB_SETBIT:
            GPIO_SetBits(GPIOB, request->wValue);
            return tud_control_status(rhport, request);

        case CMD_GPIOB_RESETBIT:
            GPIO_ResetBits(GPIOB, request->wValue);
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
            // Will STALL on invalid parameters or running transfer
            // wValue has the pin numbers (4 bit each)
            // wIndex has the SCL frequency in kHz
            if (i2c_get_result() == I2C_BUSY)
                return false;

            ret = i2c_gpio_init(request->wValue & 0xF, (request->wValue >> 4) & 0xF);
            if (!ret)
                return false;

            ret = i2c_timer_init(((unsigned)request->wIndex) * 1000);
            if (!ret)
                return false;

            return tud_control_status(rhport, request);

        case CMD_I2C_TRANSACTION:
            // wValue has the 7 bit I2C address
            // wIndex has the number of bytes to receive
            if (request->wLength > sizeof(i2c_tx_buffer) ||
                request->wIndex > sizeof(i2c_rx_buffer) - 1 || request->wValue >= 128 ||
                i2c_get_result() == I2C_BUSY)
                return false;

            // Accept the data packet
            i2c_address = request->wValue;
            i2c_n_rx = request->wIndex;

            // No TX payload. STAGE_DATA would be skipped, start the transfer here.
            if (request->wLength == 0)
                i2c_start_xfer(i2c_address, NULL, 0, &i2c_rx_buffer[1], i2c_n_rx);

            return tud_control_xfer(rhport, request, i2c_tx_buffer, request->wLength);

        // host fetches status / result of current transaction
        // host needs to fetch __at least__ i2c_n_rx + 1 bytes.
        case CMD_I2C_RESULT:
            result = i2c_get_result();
            i2c_rx_buffer[0] = result;
            if (result == I2C_BUSY)  // not ready yet
                return tud_control_xfer(rhport, request, i2c_rx_buffer, 1);
            return tud_control_xfer(rhport, request, i2c_rx_buffer, i2c_n_rx + 1);

        default:
            // Unknown RPC -> STALL (Python will raise USBError)
            return false;
        }
    } else if (stage == CONTROL_STAGE_DATA) {
        switch (request->bRequest) {
        case CMD_I2C_TRANSACTION:
            // All the I2C TX data has been received at this point. Start the I2C transaction.
            return i2c_start_xfer(
                i2c_address, i2c_tx_buffer, request->wLength, &i2c_rx_buffer[1], i2c_n_rx);

        default:
            return true;
        }
    }

    return true;
}
