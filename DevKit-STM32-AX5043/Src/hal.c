#include "hal.h"

// Sets wtimer descriptor to call 'handler' in/at 'time' (depends on 'relative')
// 'handler' is optional, if NULL, old one is used

uint8_t Supply_Voltage;

void ScheduleTask(struct wtimer_desc *desc, wtimer_desc_handler_t handler, uint8_t relative, uint32_t time)
{
    wtimer0_remove(desc);
    desc->time = time;
    if(relative)    desc->time += wtimer0_curtime();
    if(handler)     desc->handler = handler;
    wtimer0_addabsolute(desc);
}

bool CheckTask(struct wtimer_desc *desc)
{
    bool status = wtimer0_remove(desc);
    if (status) wtimer0_addabsolute(desc);
    return status;
}

__no_init uint8_t message @ (0xFF);

void HAL_RebootToBootloader()
{
    message = 0xB1;
    PCON |= 0x10;
}

void UART_Init()
{
    // Init pins
    //#ifndef AMPER
    PIN_SET_INPUT(UART_RX_DIR, UART_RX_PIN); // UART RX
    UART_RX = HIGH;

    PIN_SET_OUTPUT(UART_TX_DIR, UART_TX_PIN); // UART TX


    UART_TX = HIGH;
    //#endif


    // Init UART
    #ifdef UART_RSYSCLK
    uart_timer0_baud(CLKSRC_RSYSCLK, 115200, 26000000);//115200
    #else
    uart_timer0_baud(CLKSRC_FRCOSC, 115200, 20000000);//115200
    #endif
    uart1_init(0,8,1);
    EIE_5 = 1;//enable uart interrupt
    EIP_5 = 1;//hi priority
    // bug fix
    uart1_rxcount();
    #ifdef UART_RSYSCLK
    uart_timer0_baud(CLKSRC_RSYSCLK, 115200, 26000000);//115200
    #else
    uart_timer0_baud(CLKSRC_FRCOSC, 115200, 20000000);//115200
    #endif
}


void UART_Deinit()
{
    uart1_stop();
    PALTA &= ~0x20;
    PINSEL &= ~0x02;

    UART_RX = LOW;
    UART_TX = LOW;

}


uint8_t GetVoltageOrTemp(uint8_t val)
{
    //static uint8_t minVoltage = 0xff;
    uint8_t i = 0;
    uint32_t   VDDIOAVER = 0;
    uint16_t   VDDIO;
    uint32_t   TEMPERAVER = 0;
    uint8_t res;
    //EA = 0;
    ADCCLKSRC = 0x30;
    ADCCH1CONFIG = 0xD9;    //VddIO
    ADCCH2CONFIG = 0xD8;    //Temperature
    do
    {
        ADCCONV = 0x01;
        while(ADCCLKSRC&0x80);
        VDDIOAVER += ADCCH1VAL0;
        VDDIOAVER += (((uint16_t)ADCCH1VAL1)<<8);
        TEMPERAVER += ADCCH2VAL0;
        TEMPERAVER += (((uint16_t)ADCCH2VAL1)<<8);
    }
    while((++i)&0x0f);
    //EA = 1;
    if(val)
    {
        VDDIO = (((VDDIOAVER>>4)*1000)>>16) - 450;
        res = (VDDIO>=300)?0x80:0;
        res += VDDIO%100;
        if(val == 2)
        {
            if( Supply_Voltage > res)
            {
                Supply_Voltage = res;
            }

        }
        else Supply_Voltage = res;


    }
    else res = TEMPERAVER>>12;

    return res;

}
