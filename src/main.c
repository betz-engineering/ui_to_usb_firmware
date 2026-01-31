#include "ch32v20x.h"
#include "ch32v20x_spi.h"
#include "tusb.h"
#include "usb_interface.h"
#include <stdint.h>

static void spi_init(void) {
    SPI_InitTypeDef spi_cfg = {0};
    SPI_StructInit(&spi_cfg);
    spi_cfg.SPI_Mode = SPI_Mode_Master;
    spi_cfg.SPI_DataSize = SPI_DataSize_16b;
    spi_cfg.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  // 9 MHz I think
    SPI_Init(SPI1, &spi_cfg);
}

// Call this in your main loop
int main(void) {
    // Ensure clock is setup
    SystemCoreClockUpdate();

    spi_init();
    tud_init(BOARD_TUD_RHPORT);

    while (1) {
        tud_task();
        vendor_task();
    }
}
