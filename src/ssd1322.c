#include "ssd1322.h"
#include "ch32v20x_spi.h"
#include "core_riscv.h"
#include "main.h"
#include "ui_board.h"
#include <stdbool.h>
#include <stdint.h>

// set data/command pin of display, 0 = command, 1 = data
#define D_C(val) GPIO_WriteBit(GPIOA, PIN_D_C, val)

// Set the CS_N pin
#define CS_N(val) GPIO_WriteBit(GPIOA, PIN_CS_OLED_N, val)

// Initialization for NHD-2.8-25664UCB2 OLED display
// negative = command, positive = data
// clang-format off
static const int16_t init[] = {
    -0xFD, 0x12,     // Unlock OLED driver IC
    -0xAE,           // Display OFF (blank)
    -0x15, 0x1C, 0x5B,  // Set column address to 1C, 5B
    -0x75, 0x00, 0x3F,  // Set row address to 00, 3F
    -0xB3, 0x91,     // set clock to 80 fps
    -0xCA, 0x3F,     // Multiplex ratio, 1/64, 64 COMS enabled
    -0xA2, 0x00,     // Set offset, the display map starting line is COM0
    -0xA1, 0x00,     // Set start line position
    -0xA0, 0x14, 0x11,   // Set remap, horiz address increment, disable colum address remap,
                     //  enable nibble remap, scan from com[N-1] to COM0, disable COM split odd even
    -0xB5, 0x00,     // Disable GPIO inputs
    -0xAB, 0x01,     // Select external VDD
    -0xB4, 0xA0, 0xB5,   // Display enhancement A, 0xA0: external VSL, 0xA2: internal VSL, 0xB5: normal, 0xFD: enhanced low GS
    -0xC1, 0x7F,     // Contrast current, 256 steps, default is 0x7F
    -0xC7, 0x08,     // Master contrast current (brightness), 16 steps, default is 0x0F
    // Load custom gamma table
    -0xB8, 0, 1, 4, 8, 15, 23, 33, 45, 59, 74, 92, 111, 132, 155, 180,
    // -0xB9,           // load default gamma table
    -0xB1, 0xF4,     // Reset period / first pre-charge period Length
    // -0xD1, 0x82, 0x20    // Display enhancement B
    -0xD1, 0xa2, 0x20    // Display enhancement B
    -0xBB, 0x17,     // Pre-charge voltage
    -0xB6, 0x08,     // Second pre-charge period = 8 clks
    -0xBE, 0x04,     // VCOMH: Set Common Pins Deselect Voltage Level as 0.8 * VCC
    -0xA6,           // Normal display
    -0xA9,           // Disable partial display mode
    -0xAF            // Display ON
};
// clang-format on

static void send_cmd(uint8_t val) {
    D_C(0);
    spi_rxtx(val);
    D_C(1);
}

static void send_init(const int16_t *init, unsigned len) {
    for (unsigned i = 0; i < len; i++) {
        if (*init < 0)
            send_cmd(-(*init));
        else
            spi_rxtx(*init);
        init++;
    }
    send_cmd(0x00);  // enable custom gamma table
}

void init_ssd1322(void) {
    CS_N(0);
    D_C(1);
    send_init(init, sizeof(init) / sizeof(init[0]));
    CS_N(1);
}

void set_brightness(uint8_t val) {
    CS_N(0);
    if (val == 0) {
        send_cmd(0xAE);  // display off
    } else {
        send_cmd(0xAF);  // display on
        send_cmd(0xC7);  // set brightness (0 - 15)
        spi_rxtx(val - 1);
    }
    CS_N(1);
}

void set_inverted(bool val) {
    CS_N(0);
    send_cmd(val ? 0xA7 : 0xA6);
    CS_N(1);
}

void clear_fb(void) {
    CS_N(0);
    send_cmd(0x5C);  // write VRAM command
    for (int i = 0; i < 8192; i++)
        spi_rxtx(0);
    CS_N(1);
}

void send_fb(bool prefix_cmd, unsigned count, uint8_t *buf) {
    CS_N(0);
    if (prefix_cmd)
        send_cmd(0x5C);  // write VRAM command
    for (int i = 0; i < count; i++) {
        // spi_rxtx(*buf++);
        // doing it like this removes gaps between bytes
        while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
            ;
        SPI_I2S_SendData(SPI1, *buf++);
    }
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY))
        ;

    // Clear RXNE flag. Just in case.
    SPI_I2S_ReceiveData(SPI1);

    CS_N(1);
}

// x1, y1, x2, y2: the rectangle to update in [pixels]
// x1, y1, x2 and y2 are all inclusive!
// note that ssd1322 works with columns of 4 pixels horizontally
// so the lower 2 bits of x1 and x2 will be truncated
// data in 4 bits / pixel, 2 pixels / byte
// void send_window_4(unsigned x1, unsigned y1, unsigned x2, unsigned y2, uint8_t *data) {
//     // printf("send_window_4(%3d, %3d, %3d, %3d)\n", x1, y1, x2, y2);

//     // truncate the 2 LSBs
//     x1 >>= 2;
//     x2 >>= 2;

//     ui_phy_cs_write(SSD1322_CS);
//     send_cmd(0x15);  // Set column address range
//     spi_rxtx(0x1C + x1);
//     spi_rxtx(0x1C + x2);

//     send_cmd(0x75);  // Set row address range
//     spi_rxtx(y1);
//     spi_rxtx(y2);

//     send_cmd(0x5C);  // write VRAM
//     for (int row = y1; row <= y2; row++) {
//         uint8_t *p = &data[row * DISPLAY_WIDTH / 2 + x1 * 2];
//         for (int column = x1; column <= x2; column++) {
//             // Each column contains 4 pixels = 2 bytes
//             spi_rxtx(*p++);
//             spi_rxtx(*p++);
//         }
//     }
// }
