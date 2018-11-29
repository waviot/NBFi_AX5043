#include "main.h"
#include "radio.h"
#include "wtimer.h"
#include "string.h"
#include "rf.h"
#include "nbfi_config.h"
#include "waviotdvk.h"                                                          //added
#include "nbfi_misc.h"                                                          //added
#include "adc.h"                                                                //added

#define MODEM_ID  *((const uint32_t*)0x0801ff80)  
#define KEY  ((const uint32_t*)0x0801ff84)            

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

#define SPI_TIMEOUT	1000
#define EEPROM_INT_nbfi_data (DATA_EEPROM_BASE + 1024*5)

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
  PCB,                //tx_antenna;
  PCB,                //rx_antenna;
  TX_MAX_POWER,       //tx_pwr;
  3600*6,             //heartbeat_interval
  255,                //heartbeat_num
  0,                  //additional_flags
  NBFI_UL_FREQ_BASE,
  NBFI_DL_FREQ_BASE
};

static uint8_t nbfi_lock = 0;
extern SPI_HandleTypeDef hspi1;
extern LPTIM_HandleTypeDef hlptim1;
extern bool status_sleep;                                                       //added
struct wtimer_desc buttons_desc;

void CheckButtons(struct wtimer_desc *desc)
{
  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB1_GPIO_Port, SB1_Pin))
  {
    SetButtonFlags(SB1_PRESS);                                                  //установка флага нажатия кнопки
  }
  else
  {
    //ResetButtonFlags(SB1_PRESS);                                                //сброс флага нажатия кнопки
  }
  
  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB2_GPIO_Port, SB2_Pin))
  {
    SetButtonFlags(SB2_PRESS);                                                  //установка флага нажатия кнопки
  }
  else
  {
    //ResetButtonFlags(SB2_PRESS);                                                //сброс флага нажатия кнопки
  }
  
  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB3_GPIO_Port, SB3_Pin))
  {
    SetButtonFlags(SB3_PRESS);                                                  //установка флага нажатия кнопки
  }
  else
  {
    //ResetButtonFlags(SB3_PRESS);                                                //сброс флага нажатия кнопки
  }
  
  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SB4_GPIO_Port, SB4_Pin))
  {
    SetButtonFlags(SB4_PRESS);                                                  //установка флага нажатия кнопки
  }
  else
  {
    //ResetButtonFlags(SB4_PRESS);                                                //сброс флага нажатия кнопки
  }
}

void HAL_SYSTICK_Callback(void)
{
  if(!nbfi_lock) wtimer_runcallbacks();
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  wtimer_cc0_irq();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == AX_IRQ_Pin)
  {
    axradio_isr();
  }
  else
  {
    if(GPIO_Pin == SB1_Pin)
    {
      SetButtonFlags(SB1_INT);                                                  //установка флага нажатия кнопки
    }
    else
    {
      //ResetButtonFlags(SB1_INT);                                                //сброс флага нажатия кнопки
    }
    
    if(GPIO_Pin == SB2_Pin)
    {
      SetButtonFlags(SB2_INT);                                                  //установка флага нажатия кнопки
    }
    else
    {
      //ResetButtonFlags(SB2_INT);                                                //сброс флага нажатия кнопки
    }    
    
    if(GPIO_Pin == SB3_Pin)
    {
      SetButtonFlags(SB3_INT);                                                  //установка флага нажатия кнопки
    }
    else
    {
      //ResetButtonFlags(SB3_INT);                                                //сброс флага нажатия кнопки
    }    
    
    if(GPIO_Pin == SB4_Pin)
    {
      SetButtonFlags(SB4_INT);                                                  //установка флага нажатия кнопки
    }
    else
    {
      //ResetButtonFlags(SB4_INT);                                                //сброс флага нажатия кнопки
    }
    
    if(!status_sleep)
      ScheduleTask(&buttons_desc, &CheckButtons, RELATIVE, MILLISECONDS(50));   //опросить кнопоки через 50 мс
  }
}

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
  return HAL_GPIO_ReadPin(AX_IRQ_GPIO_Port, AX_IRQ_Pin);
}

void ax5043_spi_rx(uint8_t *pData, uint16_t Size)
{
  HAL_SPI_Receive(&hspi1, pData, Size, SPI_TIMEOUT);
}

void ax5043_spi_tx(uint8_t *pData, uint16_t Size)
{
  HAL_SPI_Transmit(&hspi1, pData, Size, SPI_TIMEOUT);
}

void ax5043_spi_tx_rx(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
  HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, Size, SPI_TIMEOUT);
}

void ax5043_spi_write_cs(uint8_t state)
{
  if (state)
    HAL_GPIO_WritePin(AX_SPI_CS_GPIO_Port, AX_SPI_CS_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(AX_SPI_CS_GPIO_Port, AX_SPI_CS_Pin, GPIO_PIN_RESET);
}

void ax5043_on_off_pwr(uint8_t pwr)
{
  //ax5043 asic external power on/off implementation might be here for hardware reset  
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
  static uint16_t prev = 0; 
  uint16_t timer = (uint16_t) hlptim1.Instance->CNT;
  if((timer < prev) && ((prev - timer) < 10000))
  {
    return prev;
  }
  prev = timer;
  return timer;
}

uint8_t wtimer_check_cc_irq(uint8_t chan)
{
  return __HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_IT_CMPM);
}


void nbfi_before_tx(NBFi_ax5043_pins_s * pins)
{
  pins->sysclk = 1; // TX Led ON
  
  if(nbfi.tx_antenna == PCB)
  {                                                                             
    pins->data = 1;
    pins->dclk = 0;
  }
  else
  {
    pins->data = 0;
    pins->dclk = 1;
  }
  pins->cfga = PA_DIFFERENTIAL;
}

void nbfi_before_rx(NBFi_ax5043_pins_s * pins)
{
  pins->sysclk = 0; // TX Led OFF
  
  if(nbfi.rx_antenna == PCB)
  {
    pins->data = 1;
    pins->dclk = 0;
  }
  else
  {
    pins->data = 0;
    pins->dclk = 1;    
  }
  pins->cfga = PA_DIFFERENTIAL;
}

void nbfi_before_off(NBFi_ax5043_pins_s * pins)
{
  pins->sysclk = 0; // TX Led OFF
  
  pins->data = 0;
  pins->dclk = 0;
  pins->antsel = 0;
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
  memcpy((void*)settings, ((const void*)EEPROM_INT_nbfi_data), sizeof(nbfi_settings_t));
}

void nbfi_write_flash_settings(nbfi_settings_t* settings)
{
  if(HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK) return;
  for(uint8_t i = 0; i != sizeof(nbfi_settings_t); i++)
  {
    if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, EEPROM_INT_nbfi_data + i, ((uint8_t *)settings)[i]) != HAL_OK) break;
  }
  HAL_FLASHEx_DATAEEPROM_Lock(); 
}

uint32_t nbfi_measure_valtage_or_temperature(uint8_t val)
{
  if(ADC_Get() == 0)
    return val ? adc_temp : adc_vdda;
  else
    return 0;
}

uint32_t nbfi_update_rtc()
{
  //you should use this callback when external RTC used
  //return rtc_counter;  
  return 0;
}

void nbfi_rtc_synchronized(uint32_t time)
{
  //you should use this callback for RTC counter correction when external RTC used
  //rtc_counter = time;
}


void nbfi_receive_complete(uint8_t * data, uint16_t length)
{
  // NBFi_Send(data, length); //loopback
}

void nbfi_lock_unlock_nbfi_irq(uint8_t lock)
{
  nbfi_lock = lock;
}

void ax5043_init(void)
{
  HAL_LPTIM_Counter_Start(&hlptim1, 0xffff);
  
  ax5043_reg_func(AXRADIO_ENABLE_GLOBAL_IRQ, (void*)ax5043_enable_global_irq);
  ax5043_reg_func(AXRADIO_DISABLE_GLOBAL_IRQ, (void*)ax5043_disable_global_irq);
  ax5043_reg_func(AXRADIO_ENABLE_IRQ_PIN, (void*)ax5043_enable_pin_irq);
  ax5043_reg_func(AXRADIO_DISABLE_IRQ_PIN, (void*)ax5043_disable_pin_irq);
  ax5043_reg_func(AXRADIO_GET_IRQ_PIN, (void*)ax5043_get_irq_pin_state);
  ax5043_reg_func(AXRADIO_SPI_RX, (void*)ax5043_spi_rx);
  ax5043_reg_func(AXRADIO_SPI_TX, (void*)ax5043_spi_tx);
  ax5043_reg_func(AXRADIO_SPI_TX_RX, (void*)ax5043_spi_tx_rx);
  ax5043_reg_func(AXRADIO_SPI_CS_WRITE, (void*)ax5043_spi_write_cs);
  ax5043_reg_func(AXRADIO_ON_OFF_PWR,(void*)ax5043_on_off_pwr);
  
  HAL_GPIO_WritePin(AX_SPI_CS_GPIO_Port, AX_SPI_CS_Pin, GPIO_PIN_SET);
  
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
  NBFI_reg_func(NBFI_BEFORE_OFF, (void*)nbfi_before_off);
  NBFI_reg_func(NBFI_RECEIVE_COMLETE, (void*)nbfi_receive_complete);
  NBFI_reg_func(NBFI_READ_FLASH_SETTINGS, (void*)nbfi_read_flash_settings);
  NBFI_reg_func(NBFI_WRITE_FLASH_SETTINGS, (void*)nbfi_write_flash_settings);
  NBFI_reg_func(NBFI_READ_DEFAULT_SETTINGS, (void*)nbfi_read_default_settings);
  NBFI_reg_func(NBFI_MEASURE_VOLTAGE_OR_TEMPERATURE, (void*)nbfi_measure_valtage_or_temperature);
  
  //register callbacks when external RTC used
  //NBFI_reg_func(NBFI_UPDATE_RTC, (void*)nbfi_update_rtc);
  //NBFI_reg_func(NBFI_RTC_SYNCHRONIZED, (void*)nbfi_rtc_synchronized);
  
  nbfi_dev_info_t info = {MODEM_ID, (uint32_t*)KEY, TX_MIN_POWER, TX_MAX_POWER, HW_ID, HW_REV, BAND, SEND_INFO_PERIOD};
  
  NBFi_Config_Set_Device_Info(&info);
  //NBFi_Clear_Saved_Configuration(); //if you need to clear previously saved nbfi configuration in EEPROM
  NBFI_Init(0); 
}