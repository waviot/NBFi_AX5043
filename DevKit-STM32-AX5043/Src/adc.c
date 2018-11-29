#include "main.h"
#include "adc.h"
#include "stm32l0xx_ll_adc.h"

#define ADC_TIMEOUT     100

int32_t adc_vdda;
int32_t adc_temp;

extern ADC_HandleTypeDef hadc;

int8_t ADC_Get(void)
{
  uint16_t timeout;
  
  timeout = ADC_TIMEOUT;
  HAL_ADC_Start(&hadc);
  while(!__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOC) && --timeout);
  if (!timeout)
    return -1;
  //if(HAL_ADC_PollForConversion(&hadc, 0) == HAL_OK)
  adc_vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(HAL_ADC_GetValue(&hadc), LL_ADC_RESOLUTION_12B);
  
  timeout = ADC_TIMEOUT;
  HAL_ADC_Start(&hadc);
  while(!__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOC) && --timeout);
  if (!timeout)
    return -1;
  //if(HAL_ADC_PollForConversion(&hadc, 0) == HAL_OK)
  adc_temp = __LL_ADC_CALC_TEMPERATURE(adc_vdda, HAL_ADC_GetValue(&hadc), LL_ADC_RESOLUTION_12B);
  
  return 0;
}