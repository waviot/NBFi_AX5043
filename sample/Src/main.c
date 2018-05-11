
#include "main.h"
#include "stm32_init.h"
#include "string.h"
#include "wtimer.h"
#include "radio.h"
#include "nbfi.h"



struct wtimer_desc test_desc;

void send_data(struct wtimer_desc *desc) {

  if(!NBFi_Packets_To_Send())
      NBFi_Send("Hello everybody!", sizeof("Hello everybody!"));
   ScheduleTask(desc, 0, RELATIVE, SECONDS(120));

}

extern uint8_t nbfi_lock;
void HAL_SYSTICK_Callback(void)
{
  if(!nbfi_lock) wtimer_runcallbacks();
}

int main(void)
{

  HAL_Init();

  SystemClock_Config();
  
  MX_GPIO_Init();
  
  MX_NVIC_Init();
  
  ax5043_init();

  ScheduleTask(&test_desc, send_data, RELATIVE, SECONDS(1));

  while (1) 
  {     
      
      NBFi_ProcessRxPackets(1);
      if (axradio_cansleep()&& NBFi_can_sleep()) 
      {
          HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
      }
  }
}



