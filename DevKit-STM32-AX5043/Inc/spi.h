#ifndef SPI_H
#define SPI_H

#include "stm32l0xx_hal.h"
#include <stdint.h>

void    SPI_Init();
uint8_t SPI_Transfer(uint8_t c);

#endif
