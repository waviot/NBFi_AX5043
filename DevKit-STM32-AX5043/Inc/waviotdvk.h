#ifndef WAVIOTDVK_H
#define WAVIOTDVK_H

#include "stm32l0xx_hal.h"
#include "stdlib.h"
#include "stdbool.h"

#define SB1_INT         (1)
#define SB2_INT         (2)
#define SB3_INT         (4)
#define SB4_INT         (8)
#define SB1_PRESS       (16)
#define SB2_PRESS       (32)
#define SB3_PRESS       (64)
#define SB4_PRESS       (128)


//typedef struct
//{
//  uint8_t button_events;
//}wtimer_desc;

//void CheckButtons(wtimer_desc *desc);
void Buttons_Process_Start();
bool GetButton1();
bool GetButton2();
bool GetButton3();
bool GetButton4();
void SetButtonFlags(uint8_t flag);
void ResetButtonFlags(uint8_t flag);
uint8_t GetButtonState();
void Backlight(bool enable);

#endif // WAVIOTDVK_H
