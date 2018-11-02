#ifndef WAVIOTDVK_H
#define WAVIOTDVK_H

#include "stm32l0xx_hal.h"
#include "stdlib.h"
#include "stdbool.h"

#define SW1     (1)
#define SW2     (2)
#define SW3     (4)
#define SW4     (8)

//typedef struct
//{
//  uint8_t button_events;
//}wtimer_desc;

//void CheckButtons(wtimer_desc *desc);
void Buttons_Init();
bool GetButton1();
bool GetButton2();
bool GetButton3();
bool GetButton4();
bool GetButtonStateChange();
void Backlight(uint8_t enable);

#endif // WAVIOTDVK_H
