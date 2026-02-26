#include "main.h"
#include "ch32v20x.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_spi.h"
#include "core_riscv.h"
#include "debug.h"
#include "system_ch32v20x.h"
#include "tusb.h"
#include "ui_board.h"
#include "usb_interface.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

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
    // --------
    //  GPIOA
    // --------
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    GPIO_InitTypeDef gpio = {0};

    // Inputs
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = PIN_SDI;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Mode = GPIO_Mode_IPD;
    gpio.GPIO_Pin = PIN_INT_IO;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // Normal Outputs
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = PIN_RES_N | PIN_D_C | PIN_CS_OLED_N | PIN_CS_IO_N | PIN_LED;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_SetBits(GPIOA, PIN_CS_OLED_N | PIN_CS_IO_N | PIN_RES_N | PIN_LED);
    GPIO_Init(GPIOA, &gpio);

    // Alternate Outputs
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = PIN_SCK | PIN_SDO;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // --------
    //  SPI
    // --------
    SPI_InitTypeDef spi_cfg = {0};
    SPI_StructInit(&spi_cfg);
    spi_cfg.SPI_Mode = SPI_Mode_Master;
    spi_cfg.SPI_DataSize = SPI_DataSize_8b;
    spi_cfg.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  // 9 MHz I think
    spi_cfg.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(SPI1, &spi_cfg);
    SPI_Cmd(SPI1, ENABLE);

    // --------
    //  USB
    // --------
    assert(SystemCoreClock == 144000000);
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div3);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);  // FSDEV (port 0)
    // RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, ENABLE);  // USBFS (port 1)
}

// Helper function to exchange one byte
uint8_t spi_rxtx(uint8_t byteToSend) {
    SPI_I2S_SendData(SPI1, byteToSend);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY))
        ;

    // Return the received data (and clear the RXNE flag)
    return SPI_I2S_ReceiveData(SPI1);
}

// static void print_state() {
//     static int enc_ = 0;
//     static uint8_t bf_ = 0;

//     int enc = get_encoder_ticks(false);
//     if (enc != enc_) {
//         printf("enc: %d\n", enc);
//         enc_ = enc;
//     }
//     int bf = get_button_flags();
//     if (bf != bf_) {
//         printf("btn: ");
//         for (int b = 5; b >= 0; b--)
//             printf(((1 << b) & bf) ? "1" : "0");
//         printf("\n");
//         bf_ = bf;
//     }
// }

// Call this in your main loop
int main(void) {
    __disable_irq();

    // Ensure clock and peripherals are setup
    SystemInit();
    peripherals_init();

    // SDI_Printf_Enable();
    SysTick_Config(SystemCoreClock / 1000);
    __enable_irq();

    // Blink the LED on boot
    for (unsigned i=0; i<6; i++) {
        GPIO_WriteBit(GPIOA, PIN_LED, i & 1);
        delay_ms(30);
    }

    // delay_ms(1000);
    // puts("Hi, this is ui_to_usb firmware!\n");

    // Init tiny-USB
    tud_init(BOARD_TUD_RHPORT);

    ui_init();

    while (1) {
        ui_board_poll();
        tud_task();
        vendor_task();
        // print_state();
    }
}
