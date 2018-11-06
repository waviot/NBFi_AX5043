#ifndef HAL_H
#define HAL_H

#include "stdbool.h"                                                            //added by me
#include "stm32l0xx_hal.h"                                                      //#include <ax8052f143.h>
#include <libmftypes.h>
#include <libmfwtimer.h>
                                                                                //#include <libmfuart.h>
                                                                                //#include <libmfuart1.h>
                                                                                //#include "defines.h"

//Choose LOWFREQ_QUARZ or LOWFREQ_RC as source for RTC clock
//#define LOWFREQ_QUARZ
#define LOWFREQ_RC

#define HIGH (1)
#define LOW (0)

#define BIT(x)  (1<<(x))
#define SET(p,x)    do{(p) |= (x);}while(0)
#define CLEAR(p,x)  do{(p) &= ~(x);}while(0)

#define PIN_SET_INPUT(p,x)  CLEAR(p, (BIT(x)))
#define PIN_SET_OUTPUT(p,x) SET(p, (BIT(x)))
#define PIN_SET_LOW(p,x)    CLEAR(p, (BIT(x)))
#define PIN_SET_HIGH(p,x)   SET(p, (BIT(x)))

#ifdef SDR_BS3
#define TCXO            PORTB_5
#define TCXO_PORT       PORTB
#define TCXO_DIR        DIRB
#define TCXO_PIN        5
#elif defined (USB_DONGLE)
#define TCXO            PORTB_2
#define TCXO_PORT       PORTB
#define TCXO_DIR        DIRB
#define TCXO_PIN        2
#else
#define TCXO            PORTB_1
#define TCXO_PORT       PORTB
#define TCXO_DIR        DIRB
#define TCXO_PIN        1
#endif

#ifdef DEVKIT
#define TX_ANT          PORTB_0
#define TX_ANT_PORT     PORTB
#define TX_ANT_DIR      DIRB
#define TX_ANT_PIN      0

#define RX_ANT          PORTB_2
#define RX_ANT_PORT     PORTB
#define RX_ANT_DIR      DIRB
#define RX_ANT_PIN      2
#endif

#ifdef AMPER_2_3
#define U3V_FL          PINB_0
#define U3V_FL_PORT     PORTB
#define U3V_FL_DIR      DIRB
#define U3V_FL_PIN      0
#endif

#ifdef FOBOS_EXT_ANT
#define RF_DIR11           PORTB_5
#define RF_DIR11_PORT      PORTB
#define RF_DIR11_DIR       DIRB
#define RF_DIR11_PIN       5
#define RF_DIR12           PORTB_4
#define RF_DIR12_PORT      PORTB
#define RF_DIR12_DIR       DIRB
#define RF_DIR12_PIN       4
#define RF_DIR21           PORTB_3
#define RF_DIR21_PORT      PORTB
#define RF_DIR21_DIR       DIRB
#define RF_DIR21_PIN       3
#define RF_DIR22           PORTB_2
#define RF_DIR22_PORT      PORTB
#define RF_DIR22_DIR       DIRB
#define RF_DIR22_PIN       2
#endif


#if defined(SDR_RASP) || defined(AMPER) || defined (BIGTIFFANY) || defined (USPD)

#define PA_EN           PORTC_1
#define PA_EN_PORT      PORTC
#define PA_EN_DIR       DIRC
#define PA_EN_PIN       1
#elif defined (USB_DONGLE)
#define PA_EN           PORTB_0
#define PA_EN_PORT      PORTB
#define PA_EN_DIR       DIRB
#define PA_EN_PIN       0
#else
#define PA_EN           PORTC_4
#define PA_EN_PORT      PORTC
#define PA_EN_DIR       DIRC
#define PA_EN_PIN       4
#endif

#if defined (USB_DONGLE)
#define REC           PORTB_1
#define REC_PORT      PORTB
#define REC_DIR       DIRB
#define REC_PIN       1

#define MODE_KEY      PINB_5
#define MODE_KEY_OUT  PORTB_5
#define MODE_KEY_DIR  DIRB
#define MODE_KEY_PIN   5

#endif // defined



#define PA_450_EN       PORTB_0
#define PA_450_EN_PORT  PORTB
#define PA_450_EN_DIR   DIRB
#define PA_450_EN_PIN   0

#define PA_450_2_EN       PORTB_3
#define PA_450_2_EN_PORT  PORTB
#define PA_450_2_EN_DIR   DIRB
#define PA_450_2_EN_PIN   3

#define PA_450_3_EN       PORTB_4
#define PA_450_3_EN_PORT  PORTB
#define PA_450_3_EN_DIR   DIRB
#define PA_450_3_EN_PIN   4

#define PA_450_4_EN       PORTB_5
#define PA_450_4_EN_PORT  PORTB
#define PA_450_4_EN_DIR   DIRB
#define PA_450_4_EN_PIN   5


#define TX870_SW1_CTRL1         PORTC_1
#define TX870_SW1_CTRL1_PORT    PORTC
#define TX870_SW1_CTRL1_DIR     DIRC
#define TX870_SW1_CTRL1_PIN     1

#define TX870_SW1_CTRL2         PORTC_2
#define TX870_SW1_CTRL2_PORT    PORTC
#define TX870_SW1_CTRL2_DIR     DIRC
#define TX870_SW1_CTRL2_PIN     2



#define PA_860_EN       PORTB_1
#define PA_860_EN_PORT  PORTB
#define PA_860_EN_DIR   DIRB
#define PA_860_EN_PIN   1

#define LCD_A0          PORTA_1
#define LCD_A0_PORT     PORTA
#define LCD_A0_DIR      DIRA
#define LCD_A0_PIN      0

#define S1_S2_IN        PINA_0
#define S1_S2_DIR       DIRA
#define S1_S2_PIN       0

#define UART_RX         PORTA_2
#define UART_RX_PORT    PORTA
#define UART_RX_DIR     DIRA
#define UART_RX_PIN     2

#define UART_TX         PORTA_5
#define UART_TX_PORT    PORTA
#define UART_TX_DIR     DIRA
#define UART_TX_PIN     5

#define TX_RX_SEL   PORTC_0
#define TX_RX_SEL_2 PORTC_2
#define TX_RX_SEL_3 PORTC_3


#ifdef SDR_BS3
#define PA_SW1           PORTC_3
#define PA_SW1_PORT      PORTC
#define PA_SW1_DIR       DIRC
#define PA_SW1_PIN       3

#define PA_SW2           PORTC_4
#define PA_SW2_PORT      PORTC
#define PA_SW2_DIR       DIRC
#define PA_SW2_PIN       4


#endif

#ifdef LOWFREQ_QUARZ
#define SECONDS(x) ((uint32_t)(x) * 8192)
#elif defined LOWFREQ_RC
                                                                                //#define SECONDS(x) ((uint32_t)(x) * 640)
#endif // LOWFREQ_QUARZ
#define MILLISECONDS(x) (SECONDS(x) / 1000)

#define ABSOLUTE    0
#define RELATIVE    1

#define NAIVE_MEMCPY(x,y,n) for(int i=0;i<(n);i++){x[i]=y[i];}

#define RemoveEvent(x)  wtimer0_remove(x)

void ScheduleTask(struct wtimer_desc *desc, wtimer_desc_handler_t handler, uint8_t relative, uint32_t time);
//bool CheckTask(struct wtimer_desc *desc);
void UART_Init();
void UART_Deinit();

uint8_t GetVoltageOrTemp(uint8_t val);
int my_sprintf(char *buf, char *fmt, ...);

extern uint8_t Supply_Voltage;
#endif // HAL_H
