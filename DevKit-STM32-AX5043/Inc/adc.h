#ifndef ADC_H
#define ADC_H

enum{
  	ADC_VCC,
	ADC_TEMP,
};

extern int32_t ADC_VDDA;
extern int32_t ADC_temp;

void ADC_init(void);
void ADC_deinit(void);
int8_t ADC_get(void);

#endif