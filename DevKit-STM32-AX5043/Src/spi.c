#include "spi.h"

void SPI_Init()
{
    
}

inline uint8_t SPI_Transfer(uint8_t c)
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

