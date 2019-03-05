#include "main.h"                                                               //added
#include "waviotdvk.h"
#include "stdbool.h"

static uint8_t button_event_flags;                                              //added


bool GetButton1()
{
  if(button_event_flags & SB1_PRESS)
  {
    //button_event_flags &= ~SB1_PRESS;
    button_event_flags = 0;
    return true;
  }
  else return false;
}

bool GetButton2()
{
  if(button_event_flags & SB2_PRESS)
  {
    //button_event_flags &= ~SB2_PRESS;
    button_event_flags = 0;
    return true;
  }
  else return false;
}

bool GetButton3()
{
  if(button_event_flags & SB3_PRESS)
  {
    //button_event_flags &= ~SB3_PRESS;
    button_event_flags = 0;
    return true;
  }
  else return false;
}

bool GetButton4()
{
  if(button_event_flags & SB4_PRESS)
  {
    //button_event_flags &= ~SB4_PRESS;
    button_event_flags = 0;
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