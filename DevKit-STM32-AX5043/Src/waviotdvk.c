#include "waviotdvk.h"
#include "stdbool.h"
#include "wtimer.h"

#define PRESS_SB 5

//static uint8_t CntPressSB1 = 0;
//static uint8_t CntPressSB2 = 0;
//static uint8_t CntPressSB3 = 0;
//static uint8_t CntPressSB4 = 0;
uint8_t button_event_flags = 0;                                                 //static uint8_t button_event_flags = 0;
struct wtimer_desc buttons_desc;
extern bool status_sleep;                                                       //added

void CheckButtons(struct wtimer_desc *desc)
{
  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB1_GPIO_Port, SB1_Pin))
  {
    button_event_flags |= SB1_PRESS;                                                  //установка флага нажатия кнопки
  }
  else
  {
    //button_event_flags &= ~SB1_PRESS;                                                 //сброс флага нажатия кнопки
  }
  
  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB2_GPIO_Port, SB2_Pin))
  {
    button_event_flags |= SB2_PRESS;                                                  //установка флага нажатия кнопки
  }
  else
  {
    //button_event_flags &= ~SB2_PRESS;                                                 //сброс флага нажатия кнопки
  }
  
  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB3_GPIO_Port, SB3_Pin))
  {
    button_event_flags |= SB3_PRESS;                                                  //установка флага нажатия кнопки
  }
  else
  {
    //button_event_flags &= ~SB3_PRESS;                                                 //сброс флага нажатия кнопки
  }
  
  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB4_GPIO_Port, SB4_Pin))
  {
    button_event_flags |= SB4_PRESS;                                                  //установка флага нажатия кнопки
  }
  else
  {
    //button_event_flags &= ~SB4_PRESS;                                                 //сброс флага нажатия кнопки
  }
  
  if(!status_sleep)
    ScheduleTask(desc, 0, RELATIVE, MILLISECONDS(100));
}

void Buttons_Process_Start()
{
  ScheduleTask(&buttons_desc, &CheckButtons, RELATIVE, MILLISECONDS(200));
}

bool GetButton1()
{
  if(button_event_flags & SB1_PRESS)
  {
    button_event_flags &= ~SB1_PRESS;
    //button_event_flags = 0;
    return true;
  }
  else return false;
}

bool GetButton2()
{
  if(button_event_flags & SB2_PRESS)
  {
    button_event_flags &= ~SB2_PRESS;
    //button_event_flags = 0;
    return true;
  }
  else return false;
}

bool GetButton3()
{
  if(button_event_flags & SB3_PRESS)
  {
    button_event_flags &= ~SB3_PRESS;
    //button_event_flags = 0;
    return true;
  }
  else return false;
}

bool GetButton4()
{
  if(button_event_flags & SB4_PRESS)
  {
    button_event_flags &= ~SB4_PRESS;
    //button_event_flags = 0;
    return true;
  }
  else return false;
}

void SetButtonFlags(uint8_t flag)
{
  button_event_flags |= flag;
}

void ResetButtonFlags(uint8_t flag)
{
  button_event_flags &= ~flag;
}

uint8_t GetButtonState()
{
  uint8_t button_state;
  
  button_state = button_event_flags;
  return button_state;
}

void Backlight(bool enable)
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