
#ifndef STM32_INIT_H_
#define STM32_INIT_H_

#include "stm32l0xx_hal.h"

extern LPTIM_HandleTypeDef hlptim1;

extern SPI_HandleTypeDef hspi2;

void SystemClock_Config(void);

void MX_GPIO_Init(void);

void MX_LPTIM1_Init(void);

void MX_SPI2_Init(void);

void MX_NVIC_Init(void);

#endif