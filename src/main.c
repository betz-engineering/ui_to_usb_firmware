#include "main.h"
#include "ch32v20x.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_spi.h"
#include "tusb.h"
#include "ui_board.h"
#include "usb_interface.h"
#include <assert.h>
#include <stdint.h>

// -----------------------------
//  Systick interrupt
// -----------------------------
volatile uint32_t system_ticks = 0;

__attribute__((interrupt)) void SysTick_Handler(void) {
    SysTick->SR = 0;
    system_ticks++;
}

static uint32_t SysTick_Config(uint32_t ticks) {
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->CTLR = 0;
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = ticks - 1;
    SysTick->CTLR = 0xF;
    return 0;
}

unsigned millis(void) { return system_ticks; }

void delay_ms(unsigned val) {
    unsigned t2 = millis() + val;
    while (millis() < t2)
        ;
}

// -----------------------------
//  USB interrupts
// -----------------------------
// Port0: USBD (fsdev)
__attribute__((interrupt)) __attribute__((used)) void USB_LP_CAN1_RX0_IRQHandler(void) {
#if CFG_TUD_WCH_USBIP_FSDEV
    tud_int_handler(0);
#endif
}

__attribute__((interrupt)) __attribute__((used)) void USB_HP_CAN1_TX_IRQHandler(void) {
#if CFG_TUD_WCH_USBIP_FSDEV
    tud_int_handler(0);
#endif
}

__attribute__((interrupt)) __attribute__((used)) void USBWakeUp_IRQHandler(void) {
#if CFG_TUD_WCH_USBIP_FSDEV
    tud_int_handler(0);
#endif
}

static void peripherals_init(void) {
    SysTick_Config(SystemCoreClock / 1000);

    // --------
    //  GPIOA
    // --------
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio = {0};

    // INT_IO = PA0
    gpio.GPIO_Pin = PIN_INT_IO;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &gpio);

    // RES_N = PA1
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = PIN_RES_N;
    GPIO_Init(GPIOA, &gpio);

    // D_C = PA2
    gpio.GPIO_Pin = PIN_D_C;
    GPIO_Init(GPIOA, &gpio);

    // CS_OLED_N = PA3
    gpio.GPIO_Pin = PIN_CS_OLED_N;
    GPIO_Init(GPIOA, &gpio);

    // CS_IO_N = PA4 = SPI_NSS
    gpio.GPIO_Pin = PIN_CS_IO_N;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    // SCK = PA5 = SPI_SCK
    gpio.GPIO_Pin = PIN_SCK;
    GPIO_Init(GPIOA, &gpio);

    // SDI = PA6 = SPI_MISO
    gpio.GPIO_Pin = PIN_SDI;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &gpio);

    // SDO = PA7 = SPI_MOSI
    gpio.GPIO_Pin = PIN_SDO;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    // --------
    //  SPI
    // --------
    SPI_InitTypeDef spi_cfg = {0};
    SPI_StructInit(&spi_cfg);
    spi_cfg.SPI_Mode = SPI_Mode_Master;
    spi_cfg.SPI_DataSize = SPI_DataSize_16b;
    spi_cfg.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  // 9 MHz I think
    SPI_Init(SPI1, &spi_cfg);

    // --------
    //  USB
    // --------
    assert(SystemCoreClock == 144000000);
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div3);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);  // FSDEV
}

// Call this in your main loop
int main(void) {
    __disable_irq();

    // Ensure clock and peripherals are setup
    SystemCoreClockUpdate();
    peripherals_init();
    __enable_irq();

    // Init tiny-USB
    tud_init(BOARD_TUD_RHPORT);

    while (1) {
        tud_task();
        vendor_task();
        ui_board_poll();
    }
}
