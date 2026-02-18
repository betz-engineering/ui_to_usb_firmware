#pragma once
#include <stdint.h>

#define PIN_LED GPIO_Pin_8
// #define PIN_LED GPIO_Pin_15  // Only for nanoCH32V203 dev board

#define PIN_INT_IO GPIO_Pin_0
#define PIN_RES_N GPIO_Pin_1
#define PIN_D_C GPIO_Pin_2
#define PIN_CS_OLED_N GPIO_Pin_3
#define PIN_CS_IO_N GPIO_Pin_4
#define PIN_SCK GPIO_Pin_5
#define PIN_SDI GPIO_Pin_6
#define PIN_SDO GPIO_Pin_7

unsigned millis(void);
void delay_ms(unsigned val);
uint8_t spi_rxtx(uint8_t byteToSend);
