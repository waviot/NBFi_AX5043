
#ifndef RADIO_H_
#define RADIO_H_

#include "axradio.h"

void ax5043_init(void);
void ax5043_spi_write_cs(uint8_t state);                                        //added by me
void ax5043_spi_tx(uint8_t *pData, uint16_t Size);                              //added by me
void ax5043_spi_rx(uint8_t *pData, uint16_t Size);                              //added by me
void ax5043_spi_tx_rx(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);       //added by me
void ax5043_enable_pin_irq(void);                                               //added by me
void ax5043_disable_pin_irq(void);                                              //added by me
void ax5043_enable_global_irq(void);                                            //added by me
void ax5043_disable_global_irq(void);                                           //added by me
void ax5043_on_off_pwr(uint8_t pwr);                                            //added by me
void wtimer_cc_irq_enable(uint8_t chan);                                        //added by me
void wtimer_cc_irq_disable(uint8_t chan);                                       //added by me
void wtimer_cc_set(uint8_t chan, uint16_t data);                                //added by me
uint16_t wtimer_cnt_get(uint8_t chan);                                          //added by me
uint8_t wtimer_check_cc_irq(uint8_t chan);                                      //added by me
uint16_t wtimer_cc_get(uint8_t chan);                                           //added by me

#endif /* RADIO_H_ */
