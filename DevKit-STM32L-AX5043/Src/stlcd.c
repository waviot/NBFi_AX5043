/*
* ST7565 LCD library
* Copyright (C) 2010 Limor Fried, Adafruit Industries
*/

#include "main.h"                                                               //added
#include <stdlib.h>
#include "wtimer.h"
#include "stlcd.h"
#include "fonts.h"

#define LCD_INIT_INTERFACE() {DIRA |= 0x02; DIRC |= 0x01; DIRB |= 0x30;}

const int pagemap[] = { 3, 2, 1, 0, 7, 6, 5, 4 };
uint8_t lcd_buffer[1024];

#define mmio(x)   (*(volatile unsigned char *)(x))

static void ST7565_InterfaceInit()
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);                //LCD_CS = 1;
}

static void ST7565_WriteCommand (uint8_t c)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);              //LCD_CS = 0;
  HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_RESET);              //LCD_A0 = 0;
  Soft_SPI_Transfer(c);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);                //LCD_CS = 1;
}

static void ST7565_WriteData (uint8_t c)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);              //LCD_CS = 0;
  HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_SET);                //LCD_A0 = 1;
  Soft_SPI_Transfer(c);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);                //LCD_CS = 1;
}

void ST7565_Init (void)
{
  ST7565_InterfaceInit();
  
  HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_SET);              //LCD_POWER = 1;
  
  // toggle RST low to reset
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);        //LCD_RST = 0;
  HAL_Delay(5);                                                                 //delay_ms(5);
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);          //LCD_RST = 1;
  HAL_Delay(5);                                                                 //delay_ms(5);
  
  // LCD bias select
  ST7565_WriteCommand (CMD_SET_BIAS_7);
  // ADC select
  ST7565_WriteCommand (CMD_SET_ADC_NORMAL);
  // SHL select
  ST7565_WriteCommand (CMD_SET_COM_NORMAL);
  // Initial display line
  ST7565_WriteCommand (CMD_SET_DISP_START_LINE);
  // turn on voltage converter (VC=1, VR=0, VF=0)
  ST7565_WriteCommand (CMD_SET_POWER_CONTROL | 0x4);
  // wait for 50% rising
  HAL_Delay(10);                                                                //delay_ms(10);
  // turn on voltage regulator (VC=1, VR=1, VF=0)
  ST7565_WriteCommand (CMD_SET_POWER_CONTROL | 0x6);
  // wait >=50ms
  HAL_Delay(50);                                                                //delay_ms(50);
  // turn on voltage follower (VC=1, VR=1, VF=1)
  ST7565_WriteCommand (CMD_SET_POWER_CONTROL | 0x7);
  // wait
  HAL_Delay(10);                                                                //delay_ms(10);
  // set lcd operating voltage (regulator resistor, ref voltage resistor)
  ST7565_WriteCommand (CMD_SET_RESISTOR_RATIO | 0x6);
  
  ST7565_WriteCommand (CMD_DISPLAY_ON);
  ST7565_WriteCommand (CMD_SET_ALLPTS_NORMAL);
  ST7565_SetBrightness (0x04);
}

void ST7565_SetBrightness (uint8_t val)
{
  ST7565_InterfaceInit();
  
  ST7565_WriteCommand (CMD_SET_VOLUME_FIRST);
  ST7565_WriteCommand (CMD_SET_VOLUME_SECOND | (val & 0x3f));
}

void ST7565_WriteBuffer (void)
{
  ST7565_InterfaceInit();
  
  for (uint16_t p = 0; p < 8; p++)
  {
    ST7565_WriteCommand (CMD_SET_PAGE | pagemap[p]);
    ST7565_WriteCommand (CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
    ST7565_WriteCommand (CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
    ST7565_WriteCommand (CMD_RMW);
    ST7565_WriteData (0x00);
    
    for (uint8_t c=0; c <LCDWIDTH; c++)
    {
      ST7565_WriteData (lcd_buffer[(LCDWIDTH*p)+c]);
      //wtimer_runcallbacks();
    }
  }
}

inline uint8_t Soft_SPI_Transfer(uint8_t c)
{
  //shiftOut(sid, sclk, MSBFIRST, c);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);              //LCD_CS = 1;
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);            //LCD_CS = 0;
  HAL_GPIO_WritePin(LCD_SCK_GPIO_Port, LCD_SCK_Pin, GPIO_PIN_RESET);          //LCD_SCK = 0
  
  for(uint8_t spi_bit_clk=8; spi_bit_clk>0; spi_bit_clk--)
  {
    if(c & (1<<(spi_bit_clk-1)))
    {
      HAL_GPIO_WritePin(LCD_MOSI_GPIO_Port, LCD_MOSI_Pin, GPIO_PIN_SET);      //LCD_MOSI = 1
    }
    else
    {
      HAL_GPIO_WritePin(LCD_MOSI_GPIO_Port, LCD_MOSI_Pin, GPIO_PIN_RESET);    //LCD_MOSI = 0
    }
    
    HAL_GPIO_WritePin(LCD_SCK_GPIO_Port, LCD_SCK_Pin, GPIO_PIN_SET);          //LCD_SCK = 1
    HAL_GPIO_WritePin(LCD_SCK_GPIO_Port, LCD_SCK_Pin, GPIO_PIN_RESET);        //LCD_SCK = 0
  }    
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);              //LCD_CS = 1;
  return 0;
}