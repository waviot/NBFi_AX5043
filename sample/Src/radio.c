#include "main.h"
#include "ax5043.h"
#include "axradio.h"
#include "radio.h"
#include "wtimer.h"
#include "string.h"
#include "nbfi.h"
#include "nbfi_config.h"
#include "stm32l0xx_hal_conf.h"

#define MODEM_ID  0x71b759  
const uint32_t KEY[8] = {0x29be5a2f,0xe2f40443,0x8a69ea5c,0x4ed60948,0x23578b53,0x0fb4556d,0x512cba03,0x83189933};  //2F5ABE294304F4E25CEA698A4809D64E538B57236D55B40F03BA2C5133991883
   
#define HW_ID     255
#define HW_REV    0
#define TX_MAX_POWER 15
#define TX_MIN_POWER 0
#define SEND_INFO_PERIOD	2592000  //one time per month
#define BAND         UL868800_DL864000


#if BAND == UL868800_DL446000
#define NBFI_UL_FREQ_BASE       (868800000 - 25000)
#define NBFI_DL_FREQ_BASE       446000000
#elif BAND == UL868800_DL864000
#define NBFI_UL_FREQ_BASE       (868800000 - 25000)
#define NBFI_DL_FREQ_BASE       864000000
#elif BAND == UL868800_DL446000_DL864000
#define NBFI_UL_FREQ_BASE       (868800000 - 25000)
#define NBFI_DL_FREQ_BASE       864000000
#elif BAND == UL867950_DL446000
#define NBFI_UL_FREQ_BASE       (867950000 - 25000)
#define NBFI_DL_FREQ_BASE       446000000
#elif BAND == UL868500_DL446000
#define NBFI_UL_FREQ_BASE       (868500000 - 25000)
#define NBFI_DL_FREQ_BASE       446000000
#elif BAND == UL868100_DL446000
#define NBFI_UL_FREQ_BASE       (868100000 - 25000)
#define NBFI_DL_FREQ_BASE       446000000
#elif BAND == UL864000_DL446000
#define NBFI_UL_FREQ_BASE       (864000000 - 25000)
#define NBFI_DL_FREQ_BASE       446000000
#elif BAND == UL863175_DL446000
#define NBFI_UL_FREQ_BASE       (863175000 - 25000)
#define NBFI_DL_FREQ_BASE       446000000
#elif BAND == UL864000_DL875000
#define NBFI_UL_FREQ_BASE       (864000000 - 25000)
#define NBFI_DL_FREQ_BASE       875000000
#endif 


const nbfi_settings_t nbfi_set_default =
{
    CRX,//mode;
    UL_DBPSK_50_PROT_D, // tx_phy_channel;
    DL_PSK_200,         // rx_phy_channel;
    HANDSHAKE_SIMPLE,
    MACK_1,             //mack_mode
    2,                  //num_of_retries;
    8,                  //max_payload_len;
    {0},                //dl_ID[3];
    {0},                //temp_ID[3];
    {0xFF,0,0},         //broadcast_ID[3];
    {0},                //full_ID[6];
    0,                  //tx_freq;
    0,                  //rx_freq;
    PCB,                //tx_antenna - no effect;
    PCB,                //rx_antenna - no effect;
    TX_MAX_POWER,       //tx_pwr;
    3600*6,             //heartbeat_interval
    255,                //heartbeat_num
    NBFI_FLG_NO_REDUCE_TX_PWR,//additional_flags
    NBFI_UL_FREQ_BASE,
    NBFI_DL_FREQ_BASE
};


extern SPI_HandleTypeDef hspi2;
extern LPTIM_HandleTypeDef hlptim1;

void ax5043_enable_global_irq(void)
{
  __enable_irq();
}

void ax5043_disable_global_irq(void)
{
  __disable_irq();
}

void ax5043_enable_pin_irq(void)
{
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void ax5043_disable_pin_irq(void)
{
  HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
}

uint8_t ax5043_get_irq_pin_state(void)
{
  return HAL_GPIO_ReadPin(AX_IRQ_PORT, AX_IRQ_PIN);
}

void ax5043_spi_rx(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  HAL_SPI_Receive(&hspi2, pData, Size, Timeout);
}

void ax5043_spi_tx(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  HAL_SPI_Transmit(&hspi2, pData, Size, Timeout);
}

void ax5043_spi_tx_rx(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
  HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, Size, Timeout);
}

void ax5043_spi_write_cs(uint8_t state)
{
  if (state)
	HAL_GPIO_WritePin(SPI2_CS_PORT, SPI2_CS_PIN, GPIO_PIN_SET);
  else
	HAL_GPIO_WritePin(SPI2_CS_PORT, SPI2_CS_PIN, GPIO_PIN_RESET);
}

void wtimer_cc_irq_enable(uint8_t chan)
{
	__HAL_LPTIM_ENABLE_IT(&hlptim1, LPTIM_IT_CMPM);
}

void wtimer_cc_irq_disable(uint8_t chan)
{
	__HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_CMPM);
}

void wtimer_cc_set(uint8_t chan, uint16_t data)
{
	hlptim1.Instance->CMP = data;
}

uint16_t wtimer_cc_get(uint8_t chan)
{
	return (uint16_t) hlptim1.Instance->CMP;
}

uint16_t wtimer_cnt_get(uint8_t chan)
{
	return (uint16_t) hlptim1.Instance->CNT;
}

uint8_t wtimer_check_cc_irq(uint8_t chan)
{
	return __HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_IT_CMPM);
}


void nbfi_before_tx()
{

}

void nbfi_before_rx()
{

}


void nbfi_read_default_settings(nbfi_settings_t* settings)
{
  for(uint8_t i = 0; i != sizeof(nbfi_settings_t); i++)
  {
    ((uint8_t *)settings)[i] = ((uint8_t *)&nbfi_set_default)[i];
  }
}


void  nbfi_read_flash_settings(nbfi_settings_t* settings) 
{
  memset((void*)settings, 0xff, sizeof(nbfi_settings_t));
}

void nbfi_write_flash_settings(nbfi_settings_t* settings)
{

}

uint32_t nbfi_measure_valtage_or_temperature(uint8_t val)
{
	return 0;
}

void nbfi_receive_complete(uint8_t * data, uint16_t length)
{

  NBFi_Send(data, length); //loopback
  
}

void ax5043_init(void)
{
       
	ax5043_reg_func(AXRADIO_ENABLE_GLOBAL_IRQ, (void*)ax5043_enable_global_irq);
	ax5043_reg_func(AXRADIO_DISABLE_GLOBAL_IRQ, (void*)ax5043_disable_global_irq);
	ax5043_reg_func(AXRADIO_ENABLE_IRQ_PIN, (void*)ax5043_enable_pin_irq);
	ax5043_reg_func(AXRADIO_DISABLE_IRQ_PIN, (void*)ax5043_disable_pin_irq);
	ax5043_reg_func(AXRADIO_GET_IRQ_PIN, (void*)ax5043_get_irq_pin_state);
	ax5043_reg_func(AXRADIO_SPI_RX, (void*)ax5043_spi_rx);
	ax5043_reg_func(AXRADIO_SPI_TX, (void*)ax5043_spi_tx);
	ax5043_reg_func(AXRADIO_SPI_TX_RX, (void*)ax5043_spi_tx_rx);
	ax5043_reg_func(AXRADIO_SPI_CS_WRITE, (void*)ax5043_spi_write_cs);

	HAL_GPIO_WritePin(SPI2_CS_PORT, SPI2_CS_PIN, GPIO_PIN_SET);

	wtimer_reg_func(WTIMER_GLOBAL_IRQ_ENABLE, (void*)ax5043_enable_global_irq);
	wtimer_reg_func(WTIMER_GLOBAL_IRQ_DISABLE, (void*)ax5043_disable_global_irq);
	wtimer_reg_func(WTIMER_CC_IRQ_ENABLE, (void*)wtimer_cc_irq_enable);
	wtimer_reg_func(WTIMER_CC_IRQ_DISABLE, (void*)wtimer_cc_irq_disable);
	wtimer_reg_func(WTIMER_SET_CC, (void*)wtimer_cc_set);
	wtimer_reg_func(WTIMER_GET_CC, (void*)wtimer_cc_get);
	wtimer_reg_func(WTIMER_GET_CNT, (void*)wtimer_cnt_get);
	wtimer_reg_func(WTIMER_CHECK_CC_IRQ, (void*)wtimer_check_cc_irq);

	wtimer_init();

	NBFI_reg_func(NBFI_BEFORE_TX, (void*)nbfi_before_tx);
	NBFI_reg_func(NBFI_BEFORE_RX, (void*)nbfi_before_rx);
	NBFI_reg_func(NBFI_RECEIVE_COMLETE, (void*)nbfi_receive_complete);
	NBFI_reg_func(NBFI_READ_FLASH_SETTINGS, (void*)nbfi_read_flash_settings);
	NBFI_reg_func(NBFI_WRITE_FLASH_SETTINGS, (void*)nbfi_write_flash_settings);
        NBFI_reg_func(NBFI_READ_DEFAULT_SETTINGS, (void*)nbfi_read_default_settings);
	NBFI_reg_func(NBFI_MEASURE_VOLTAGE_OR_TEMPERATURE, (void*)nbfi_measure_valtage_or_temperature);

	nbfi_dev_info_t info = {MODEM_ID, (uint32_t*)KEY, TX_MIN_POWER, TX_MAX_POWER, HW_ID, HW_REV, BAND, SEND_INFO_PERIOD};

	NBFi_Config_Set_Device_Info(&info);

	NBFI_Init(0);
  
}



