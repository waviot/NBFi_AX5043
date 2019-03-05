/*
 * ST7565 LCD library
 * Copyright (C) 2010 Limor Fried, Adafruit Industries
 */

#include <stdlib.h>
#include <string.h>
#include "stm32l0xx_hal.h"
#include "glcd.h"
#include "stlcd.h"
#include "fonts.h"

const FONT_INFO *FNT = 0;

void LCD_SetFont(const FONT_INFO *fnt)
{
    FNT = fnt;
}

void LCD_DrawChar(uint16_t *x, uint16_t y, uint8_t ch, uint8_t color)
{
    const FONT_CHAR_INFO *charInfo = NULL;
    uint16_t x_coord,y_coord,coord;

    if(ch == ' ')// Space, TODO optimise
    {
        *x += FNT->spacePixels * 4;
        return;
    }

    if(FNT->charInfo == NULL)
    {
        for(uint8_t i=0; i<sizeof(FNT->BlockLookup);i++)
        {
            if((ch >= FNT->BlockLookup[i].startChar) && (ch <= FNT->BlockLookup[i].endChar))
            {
                charInfo = FNT->BlockLookup[i].charInfo;
                ch -= FNT->BlockLookup[i].startChar;
                break;
            }
        }
        if(charInfo == NULL) return; // If char not found in index
    }
    else
    {
        if((ch >= FNT->startChar) && (ch <= FNT->endChar))
        {
            charInfo = FNT->charInfo;
            ch -= FNT->startChar;
        }
        else return;
    }

    if(color != COLOR_NONE)
    {
        for(uint8_t X=0; X<charInfo[ch].widthBits; X++)
        {
            for(uint8_t Y = 0; Y<FNT->heightChar; Y++)
            {
                // Fill lcd_bufer with font data, considering byte misalignment
                x_coord = *x + X;
                y_coord = y/8 + Y;
                coord = x_coord+(y_coord*LCDWIDTH);
                if((x_coord < LCDWIDTH) && (coord<sizeof(lcd_buffer)))
                    if(color == COLOR_FILL)
                        lcd_buffer[coord] |= (FNT->data[charInfo[ch].offset+(FNT->heightChar*X)+(FNT->heightChar-Y-1)])>>(y%8);
                    else // if(color == COLOR_INVERT)
                        lcd_buffer[coord] ^= (FNT->data[charInfo[ch].offset+(FNT->heightChar*X)+(FNT->heightChar-Y-1)])>>(y%8);

                coord += LCDWIDTH; // Next row
                if((x_coord < LCDWIDTH) && (coord<sizeof(lcd_buffer)))
                    if(color == COLOR_FILL)
                        lcd_buffer[coord] |= (FNT->data[charInfo[ch].offset+(FNT->heightChar*X)+(FNT->heightChar-Y-1)])<<(8-(y%8));
                    else // if(color == COLOR_INVERT)
                        lcd_buffer[coord] ^= (FNT->data[charInfo[ch].offset+(FNT->heightChar*X)+(FNT->heightChar-Y-1)])<<(8-(y%8));
            }
        }
    }

    *x += charInfo[ch].widthBits + FNT->spacePixels;
}

void LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint8_t color, uint8_t align)
{
//    // Check if X is pointer or variable
//    uint16_t X;
//    if((uint32_t)x < 0x100)
//    {
//        X = *x;
//        x = &X;
//    }

    if(align == ALIGN_LEFT)
    {
        while (*str)
        {
            char ch = *str++;
            LCD_DrawChar (&x, y, ch, color);
        }
    }
    else // if(align == ALIGN_RIGHT || align == ALIGN_CENTERED)
    {
        // Calculate string length first
        const char *str_temp = str;
        uint16_t x_temp = 0;
        while (*str_temp)
        {
            char ch = *str_temp++;
            LCD_DrawChar (&x_temp, y, ch, COLOR_NONE);
        }

        if(align == ALIGN_CENTERED) x_temp /= 2;

        // Then draw string with calculated length offset
        x -= x_temp;
        while (*str)
        {
            char ch = *str++;
            LCD_DrawChar (&x, y, ch, color);
        }
        x -= x_temp;
    }
}

void LCD_ClearBuffer(bool all)
{
	memset(lcd_buffer, all, sizeof(lcd_buffer));
}

void LCD_SetPixel(uint8_t x, uint8_t y, uint8_t color)
{
	if ((x >= LCDWIDTH) || (y >= LCDHEIGHT))
		return;

	// x is which column
	if (color == COLOR_FILL)
		lcd_buffer[x+ (y/8)*LCDWIDTH] |= _BV(7-(y%8));
	else if(color == COLOR_NONE)
		lcd_buffer[x+ (y/8)*LCDWIDTH] &= ~_BV(7-(y%8));
    else if(color == COLOR_INVERT)
		lcd_buffer[x+ (y/8)*LCDWIDTH] ^= _BV(7-(y%8));
}

void LCD_FillRect (uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    // stupidest version - just pixels - but fast with internal buffer!
    for (uint8_t i=x; i <x+w; i++)
    {
        for (uint8_t j=y; j <y+h; j++)
        {
            LCD_SetPixel (i, j, color);
        }
    }
}

void LCD_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    // stupidest version - just pixels - but fast with internal buffer!
    for (uint8_t i=x; i <x+w; i++)
    {
        LCD_SetPixel (i, y, color);
        LCD_SetPixel (i, y+h-1, color);
    }
    for (uint8_t i=y; i<y+h; i++)
    {
        LCD_SetPixel (x, i, color);
        LCD_SetPixel (x+w-1, i, color);
    }
}

//void LCD_DrawNum(uint16_t* x, uint16_t y, uint32_t __generic val, uint8_t nrdig1, uint8_t flags1, uint8_t align )
//{
//    char __autodata buf[10];
//    uint16_t __autodata index = 0;
//    char __autodata ch = 0;
//    uint8_t __autodata d;
//    uint8_t __autodata cnt = 10;
//    uint8_t __autodata flags = flags1;
//    uint8_t __autodata nrdig = nrdig1;
//
//    if ((flags & WRNUM_SIGNED) && ((int32_t)val) < 0)
//    {
//        val = -val;
//        ch = '-';
//    }
//    else if ((flags & WRNUM_ZEROPLUS) || ((flags & WRNUM_PLUS) && val))
//    {
//        ch = '+';
//    }
//    if (ch && nrdig > 0)
//        --nrdig;
//    if (flags & WRNUM_TSDSEP)
//    {
//        if (nrdig > 9)
//            --nrdig;
//        if (nrdig > 6)
//            --nrdig;
//        if (nrdig > 3)
//            --nrdig;
//    }
//    flags &= WRNUM_MASK;
//    if (cnt < nrdig)
//        cnt = nrdig;
//    do
//    {
//        d = cnt;
//        val = libmf_num32_digit(val, (uint8_t __data *)&d);
//        if (!d && cnt != 1 && !(flags & WRNUM_DIGCONT))
//        {
//            if (cnt > nrdig)
//                continue;
//            if (!(flags & WRNUM_PADZERO))
//            {
//                if (!(flags & WRNUM_DIGSET))
//                {
//                    nrdig = cnt;
//                    flags |= WRNUM_DIGSET;
//                }
//                buf[index++] = ' ';//LCD_DrawChar(x,y,' ', 1);
//                if ((flags & WRNUM_TSDSEP) && (cnt == 4 || cnt == 7 || cnt == 10))
//                {
//                    buf[index++] = ' ';//LCD_DrawChar(x,y,' ', 1);
//                    ++nrdig;
//                }
//                continue;
//            }
//        }
//        if (!(flags & WRNUM_DIGCONT))
//        {
//            if (!(flags & WRNUM_DIGSET))
//                nrdig = cnt;
//            flags |= WRNUM_PADZERO | WRNUM_DIGCONT | WRNUM_DIGSET;
//            if (ch)
//            {
//                buf[index++] = ch;//LCD_DrawChar(x,y,ch, 1);
//                ++nrdig;
//            }
//        }
//        buf[index++] = '0' + d;//LCD_DrawChar(x,y,'0' + d, 1);
//        if ((flags & WRNUM_TSDSEP) && (cnt == 4 || cnt == 7 || cnt == 10))
//        {
//            buf[index++] = '\'';//LCD_DrawChar(x,y,'\'', 1);
//            ++nrdig;
//        }
//    }
//    while (--cnt);
//    buf[index] = 0;
//
//    LCD_DrawString(x, y, buf, 1, align);
//}

void LCD_WriteBuffer()
{
  ST7565_WriteBuffer();
}

void LCD_Init()
{
  ST7565_Init();
  ST7565_WriteBuffer();
}

/*****************************************************************************
 *
 *****************************************************************************/

