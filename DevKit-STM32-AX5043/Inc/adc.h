#ifndef ADC_H
#define ADC_H

enum
{
  ADC_VDDA,
  ADC_TEMP
};

extern int32_t adc_vdda;
extern int32_t adc_temp;

int8_t ADC_Get(void);

#endif