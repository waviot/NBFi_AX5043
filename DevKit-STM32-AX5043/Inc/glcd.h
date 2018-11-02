#ifndef GLCD_H
#define GLCD_H

#include "stdbool.h"
#include "fonts.h"

extern uint32_t libmf_num32_digit(uint32_t v, uint8_t *dp);

#define WRNUM_MASK    (WRNUM_SIGNED | WRNUM_PLUS | WRNUM_ZEROPLUS | WRNUM_PADZERO | WRNUM_TSDSEP | WRNUM_LCHEX)
#define WRNUM_DIGCONT 0x80
#define WRNUM_DIGSET  0x40
#define _BV(x) (1<<(x))

enum
{
    COLOR_NONE,
    COLOR_FILL,
    COLOR_INVERT
};

enum
{
    ALIGN_LEFT,
    ALIGN_RIGHT,
    ALIGN_CENTERED
};

void LCD_SetPixel (uint8_t x, uint8_t y, uint8_t color);
void LCD_DrawRect (uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void LCD_FillRect (uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void LCD_SetFont(const FONT_INFO *fnt);
void LCD_DrawChar(uint16_t *x, uint16_t y, uint8_t ch, uint8_t color);
void LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint8_t color, uint8_t align);
void LCD_ClearBuffer(bool all);
void LCD_DrawNum(uint16_t* x, uint16_t y, uint32_t val, uint8_t nrdig1, uint8_t flags1, uint8_t align );
void LCD_WriteBuffer();
void LCD_Init();
#endif
