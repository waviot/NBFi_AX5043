#include "waviotdvk.h"
#include "stdbool.h"

#define PRESS_SB 5

static uint8_t CntPressSB1 = 0;
static uint8_t CntPressSB2 = 0;
static uint8_t CntPressSB3 = 0;
static uint8_t CntPressSB4 = 0;
static uint8_t button_events = 0;
static uint8_t button_state_change = 0;
//struct wtimer_desc buttons_desc;

bool GetButton1()
{
  if(button_events & SW1) 
  {
    //button_events &= ~SW1;
    //button_events = 0;
    return true;
  }
  else return false;
}

bool GetButton2()
{
  if(button_events & SW2)
  {
    //button_events &= ~SW2;
    //button_events = 0;
    return true;
  }
  else return false;
}

bool GetButton3()
{
  if(button_events & SW3)
  {
    //button_events &= ~SW3;
    //button_events = 0;
    return true;
  }
  else return false;
}

bool GetButton4()
{
  if(button_events & SW4)
  {
    //button_events &= ~SW4;
    //button_events = 0;
    return true;
  }
  else return false;
}

bool GetButtonStateChange()
{
  button_state_change = button_events;                                  //сохраняем предыдущую конфигурация кнопок
  
  if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB1_GPIO_Port, SB1_Pin))       //если кнопка SB1 была нажата
  {
    CntPressSB1++;		                                        //Увеличиваем счётчик циклов задержки нажатия кнопки
    HAL_Delay(10);						        //Задержка 10 мс
    
    if(CntPressSB1>PRESS_SB)                                            //Если больше 5 циклов
    {
      button_events |= SW1;                                             //устанавливаем флаг нажатия кнопки
    }
  }
  else
  {
    button_events &= ~SW1;                                              //сбрасываем флаг нажатия кнопки
    CntPressSB1=0;		                                        //обнуляем счётчик
  }

  if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB2_GPIO_Port, SB2_Pin))       //если кнопка SB2 была нажата
  {
    CntPressSB2++;		                                        //Увеличиваем счётчик циклов задержки нажатия кнопки
    HAL_Delay(10);						        //Задержка 10 мс
    
    if(CntPressSB2>PRESS_SB)                                            //Если больше 5 циклов
    {
      button_events |= SW2;                                             //устанавливаем флаг нажатия кнопки
    }
  }
  else
  {
    button_events &= ~SW2;                                              //сбрасываем флаг нажатия кнопки
    CntPressSB2=0;	                                                //обнуляем счётчик
  }

  if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB3_GPIO_Port, SB3_Pin))       //если кнопка SB3 была нажата
  {
    CntPressSB3++;		                                        //Увеличиваем счётчик циклов задержки нажатия кнопки
    HAL_Delay(10);						        //Задержка 10 мс
    
    if(CntPressSB3>PRESS_SB)	                                        //Если больше 5 циклов
    {
      button_events |= SW3;                                             //устанавливаем флаг нажатия кнопки
    }
  }
  else
  {
    button_events &= ~SW3;                                              //сбрасываем флаг нажатия кнопки
    CntPressSB3=0;	                                                //обнуляем счётчик
  }
  
  if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB4_GPIO_Port, SB4_Pin))       //если кнопка SB4 была нажата
  {
    CntPressSB4++;		                                        //Увеличиваем счётчик циклов задержки нажатия кнопки
    HAL_Delay(10);						        //Задержка 10 мс
    
    if(CntPressSB4>PRESS_SB)	                                        //Если больше 5 циклов
    {
      button_events |= SW4;                                             //устанавливаем флаг нажатия кнопки
    }
  }
  else
  {
    button_events &= ~SW4;                                              //сбрасываем флаг нажатия кнопки
    CntPressSB4=0;	                                                //обнуляем счётчик
  }
  
  if(button_state_change != button_events)
  {
    button_state_change=button_events;
    return true;
  }
  else
    return false;
}

void Backlight(uint8_t enable)
{
  if(enable)
  {
    HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_RESET);      //BACKLIGHT ON
  }
  else
  {
    HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET);        //BACKLIGHT OFF
  }
}