#include "stm32l0xx_hal.h"
//#include <libmfradio.h>
//#include <libmfflash.h>
#include <libmfwtimer.h>
//#include <libmfcrc.h>
//#include <libmfadc.h>
#include "nbfi_misc.h"

                                                                                //#include "nbfi.h"
#include "rf.h"
#include "nbfi_config.h"
//#include "application.h"
#include "slip.h"
#include "hal.h"
#include "time.h"
#include "waviotdvk.h"
#include "gui.h"
#include "glcd.h"
#include "fonts.h"

uint8_t *last_rx_pkt;
uint8_t last_rx_pkt_len;

struct wtimer_desc gui_handler;
void (*current_handler)(void);

char textbuf[30]; // for formatted strings

void GUI_DrawButtonR(const char *label, uint8_t state);
void GUI_DrawButtonL(const char *label, uint8_t state);

void MainHandler();
void TestsHandler();
void NBFiTxHandler();
void NBFiRxHandler();
void RSSiHandler();
//void CarrierHandler();
void InfoHandler();
void NBFiQuality();
void DevInfoHandler();
void NBFiStats();
void SettingsHandler();

//Text labels
const char label_next[] =    "Next";
const char label_enter[] =   "Enter";
const char label_start[] =   "Start";
const char label_stop[] =    "Stop";
const char label_cancel[] =  "Cancel";
const char label_edit[] =    "Edit";
const char label_dec[] =     "<";
const char label_inc[] =     ">";
const char label_reflash[] = "Reflash";

const gui_entry_t main_table[]=
{
  {"Tests",               &TestsHandler},
  {"Settings",            &SettingsHandler},
  {"Info",                &InfoHandler},
};

void MainHandler()
{
  static uint8_t state = 0;

  // Caption
  LCD_DrawString(0,(uint16_t)-6,"../", COLOR_FILL, ALIGN_LEFT);
  
  // Entry list
  for(int i=0; i<TABLE_SIZE(main_table); i++)
  {
    LCD_DrawString(20,(i*9)+5,main_table[i].label, COLOR_FILL, ALIGN_LEFT);
  }
  
  // Cursor
  LCD_DrawString(10,(state*9)+5,">", COLOR_FILL, ALIGN_LEFT);
  
  // Button processing
  if(GetButton1())
  {
    GUI_DrawButtonL(label_next, 1);
    state++;
    state = state % TABLE_SIZE(main_table);
  }
  else
  {
    GUI_DrawButtonL(label_next, 0);
  }
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_enter, 1);
    current_handler = main_table[state].handler;
  }
  else
  {
    GUI_DrawButtonR(label_enter, 0);
  }
}

const gui_entry_t packet_tests_table[]=
{
  //{"NBFi-D",          &TestsHandler},
  {"NBFi TX",            &NBFiTxHandler},
  {"NBFi RX",            &NBFiRxHandler},
  //{"Carrier TX",       &CarrierHandler},
  //{"Hopping",         &NBFiHoppingHandler},
  {"RSSi",            &RSSiHandler},
};

void TestsHandler()
{
  static uint8_t state_h = 0;

  // Caption
  LCD_DrawString(0,(uint16_t)-6,"../Tests", COLOR_FILL, ALIGN_LEFT);
  
  // Entry list
  for(int i=0; i<TABLE_SIZE(packet_tests_table); i++)
  {
    LCD_DrawString(20,(i*9)+5,packet_tests_table[i].label, COLOR_FILL, ALIGN_LEFT);
  }
  
  // Cursor
  LCD_DrawString(10,(state_h*9)+5,">", COLOR_FILL, ALIGN_LEFT);
  
  // Button processing
  if(GetButton1())
  {
    GUI_DrawButtonL(label_next, 1);
    state_h++;
    state_h = state_h % TABLE_SIZE(packet_tests_table);
  }
  else
  {
    GUI_DrawButtonL(label_next, 0);
  }
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_enter, 1);
    current_handler = packet_tests_table[state_h].handler;
  }
  else
  {
    GUI_DrawButtonR(label_enter, 0);
  }
  
  if(GetButton3())
  {
    state_h = 0;
    current_handler = &MainHandler;
  }
}

const char *gui_packet_state[]=
{
  "FREE",
  "ALLOCATED",
  "QUEUED",
  "TRANSMIT",
  "PROCESSING",
  "DL DELAY",
  "RECEIVE",
  "WAIT ACK",
  "GOT DL",
  "TIMEOUT"
};

//extern received_packet_t __xdata* rx_pkt;
extern uint8_t rx_pkt_len;
extern int16_t rssi;
extern int16_t offset;

extern struct axradio_callback_receive axradio_cb_receive;

const gui_entry_t nbfi_tx_table[]=
{
  {"Send short packet", 0},
  {"Send long packet",  0},
};

void NBFiTxHandler()
{
  static uint8_t state_n = 0;
  //static nbfi_packet_t __xdata* pkt;
  
  //static uint8_t __xdata dl_result = 0;
  
  static uint8_t test_pkt[8] = {0,0xDE,0xAD,0xBE,0xEF,0x12,0x34,0x56};
  static uint8_t test_pkt_long[] = "Do you like this weather? I saw a politician with his hands in his own pockets.\n";
  
  // Caption
  LCD_DrawString(0,(uint16_t)-6,"../Tests/NBFi TX", COLOR_FILL, ALIGN_LEFT);
  
  
  // Entry list
  for(int i=0; i<TABLE_SIZE(nbfi_tx_table); i++)
  {
    LCD_DrawString(20,(i*9)+5,nbfi_tx_table[i].label, COLOR_FILL, ALIGN_LEFT);
  }
  
  // Cursor
  LCD_DrawString(10,(state_n*9)+5,">", COLOR_FILL, ALIGN_LEFT);
  
  // Button processing
  if(GetButton1())
  {
    GUI_DrawButtonL(label_next, 1);
    state_n++;
    state_n = state_n % TABLE_SIZE(nbfi_tx_table);
  }
  else
  {
    GUI_DrawButtonL(label_next, 0);
  }
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_enter, 1);
    switch(state_n)
    {
    case 0:
      NBFi_Send(test_pkt, sizeof(test_pkt));
      break;
    case 1:
      NBFi_Send(test_pkt_long, sizeof(test_pkt_long));
      break;
    }
    //current_handler = info_table[state_n].handler;
  }
  else
  {
    GUI_DrawButtonR(label_enter, 0);
  }
  
  LCD_DrawString(10,36,"UL enqueued:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", NBFi_Packets_To_Send());
  LCD_DrawString(127,36, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  if(GetButton3())
  {
    state_n = 0;
    current_handler = &TestsHandler;
  }
}

void NBFiRxHandler()
{
  LCD_DrawString(0,(uint16_t)-6,"../Tests/NBFi RX", COLOR_FILL, ALIGN_LEFT);

  LCD_DrawString(2,5,"Last received HEX:", COLOR_FILL, ALIGN_LEFT);
  
  for(int i=0; i<last_rx_pkt_len; i++)//-1 for 1st CMD byte
  {
    if(2*i > 22) break;
    my_sprintf(textbuf+(2*i), "%02x", ((uint8_t*)last_rx_pkt)[i]); // +1 for 1st CMD byte
  }
  LCD_DrawString(2, 15, textbuf, COLOR_FILL, ALIGN_LEFT);
  
  LCD_DrawString(2,25,"ASCII:", COLOR_FILL, ALIGN_LEFT);
  
  for(int i=0; i<last_rx_pkt_len; i++)//-1 for 1st CMD byte
  {
    if(i > 22) break;
    my_sprintf(textbuf+(i), "%c", ((uint8_t*)last_rx_pkt)[i]); // +1 for 1st CMD byte
  }
  
  LCD_DrawString(2, 35, textbuf, COLOR_FILL, ALIGN_LEFT);
  
  LCD_DrawString(2,45,"RSSI:", COLOR_FILL, ALIGN_LEFT);
  
  my_sprintf(textbuf, "%d dBm", nbfi_state.last_rssi);
  
  LCD_DrawString(127,45, textbuf, COLOR_FILL, ALIGN_RIGHT);
  /*   my_sprintf(textbuf, "%d dBm", noise);
  LCD_DrawString(127,5, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,15,"UL aver. SNR:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", nbfi_state.aver_tx_snr);
  LCD_DrawString(127,15, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,25,"DL aver. SNR:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", nbfi_state.aver_rx_snr);
  LCD_DrawString(127,25, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,35,"UL bitrate:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", NBFi_Phy_To_Bitrate(nbfi.tx_phy_channel));
  LCD_DrawString(127,35, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,45,"DL bitrate:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%i", NBFi_Phy_To_Bitrate(nbfi.rx_phy_channel));
  LCD_DrawString(127,45, textbuf, COLOR_FILL, ALIGN_RIGHT);*/
  
  if(GetButton3())
  {
    current_handler = &TestsHandler;
  }
}

/*void NBFiHoppingHandler()
{
    static uint8_t __xdata state = 0;

    // Caption
    LCD_DrawString(0,-6,"../Tests/Hopping", COLOR_FILL, ALIGN_LEFT);

    switch(state)
    {
    case 0:
        LCD_DrawString(10,5,"Ready", COLOR_FILL, ALIGN_LEFT);
        break;
    case 1:
        LCD_DrawString(10,5,"Sending", COLOR_FILL, ALIGN_LEFT);

        NBFi_TX_Hopping();
        break;
    }

    // Button processing
    if(GetButton1())
    {
        GUI_DrawButtonL(label_start, 1);
        state = 1;
    }
    else
    {
        GUI_DrawButtonL(label_start, 0);
    }

    if(GetButton2())
    {
        GUI_DrawButtonR(label_cancel, 1);
        state = 0;
    }
    else
    {
        GUI_DrawButtonR(label_cancel, 0);
    }

    if(GetButton3())
    {
        current_handler = &TestsHandler;
    }
}*/

void RSSiHandler()
{
  static uint8_t state = 0;
  static int16_t current_rssi = 0;

  switch(state)
  {
  case 0:
    LCD_DrawString(10,5,"RX disabled", COLOR_FILL, ALIGN_LEFT);
    break;
  case 1:
    LCD_DrawString(10,5,"RX enabled", COLOR_FILL, ALIGN_LEFT);
    current_rssi = (int8_t)AX5043_RSSI - (int16_t)64;
    my_sprintf(textbuf, "RSSi = %i dBm", current_rssi);
    LCD_DrawString(10,23,textbuf, COLOR_FILL, ALIGN_LEFT);
    break;
  }
  
  // Caption
  LCD_DrawString(0,(uint16_t)-6,"../Tests/RSSi", COLOR_FILL, ALIGN_LEFT);
  
  // Button processing
  if(GetButton1())
  {
    GUI_DrawButtonL(label_start, 1);
    RF_Init(DL_PSK_200, PCB, 0, 446000000);
    state = 1;
  }
  else
  {
    GUI_DrawButtonL(label_start, 0);
  }
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_cancel, 1);
    RF_Deinit();
    state = 0;
  }
  else
  {
    GUI_DrawButtonR(label_cancel, 0);
  }
  
  if(GetButton3())
  {
    current_handler = &TestsHandler;
  }
}

/*
void CarrierHandler()
{
    static uint8_t __xdata state_c = 0;
    static uint32_t __xdata freq = 868800000;
    // Caption
    LCD_DrawString(0,-6,"../Tests/Carrier TX", COLOR_FILL, ALIGN_LEFT);

    switch(state_c)
    {
    case 0:
        freq = 868800000;
        RF_Init(UL_DBPSK_50_PROT_C, nbfi.tx_antenna, nbfi.tx_pwr, freq);
        axradio_set_mode(AXRADIO_MODE_CW_TRANSMIT);
        state_c = 1;
        break;
    case 1:
        break;
    }

    my_sprintf(textbuf, "Freq: %lu Hz", freq);
    LCD_DrawString(10,5,textbuf, COLOR_FILL, ALIGN_LEFT);

    // Button processing
    if(GetButton1())
    {
        GUI_DrawButtonL(label_dec, 1);
        freq -= 1;

        RF_SetFreq(freq);
        axradio_set_channel(0);
    }
    else
    {
        GUI_DrawButtonL(label_dec, 0);
    }

    if(GetButton2())
    {
        GUI_DrawButtonR(label_inc, 1);
        freq += 1;

        RF_SetFreq(freq);
        axradio_set_channel(0);
    }
    else
    {
        GUI_DrawButtonR(label_inc, 0);
    }

    if(GetButton3())
    {
        state_c = 0;
        RF_Deinit();
        current_handler = &MainHandler;
    }
}
*/
const char fmt_uint8_t[] = "%u";
const char fmt_int8_t[] = "%d";
const char fmt_uint32_t[] = "%d";
const char fmt_str[] = "%s";

void SettingsHandler()
{
  static uint8_t state_s = 0;
  static uint8_t edit = 0;
  
  // Caption
  LCD_DrawString(0,(uint16_t)-6,"../Settings", COLOR_FILL, ALIGN_LEFT);
  
  // Entry list
  for(int i=0; i<GUI_NUM_LINES; i++)                                            //for(int i=0; i<4 && i<GUI_NUM_LINES; i++)
  {
    switch(i)
    {
    case 0:
      /*LCD_DrawString(10,(i*9)+5, "TX Power", COLOR_FILL, ALIGN_LEFT);
      my_sprintf(textbuf, "%d", nbfi.tx_pwr);
      LCD_DrawString(127,(i*9)+5, textbuf, COLOR_FILL, ALIGN_RIGHT);*/
      LCD_DrawString(10,(i*9)+5, "TX BitRate", COLOR_FILL, ALIGN_LEFT);
      my_sprintf(textbuf, "%d", NBFi_Phy_To_Bitrate(nbfi.tx_phy_channel));
      LCD_DrawString(127,(i*9)+5, textbuf, COLOR_FILL, ALIGN_RIGHT);
      break;
    case 1:
      LCD_DrawString(10,(i*9)+5, "TX Antenna", COLOR_FILL, ALIGN_LEFT);
      my_sprintf(textbuf, "%s", nbfi.tx_antenna?"SMA":"PCB");
      LCD_DrawString(127,(i*9)+5, textbuf, COLOR_FILL, ALIGN_RIGHT);
      break;
    case 2:
      LCD_DrawString(10,(i*9)+5, "RX Antenna", COLOR_FILL, ALIGN_LEFT);
      my_sprintf(textbuf, "%s", nbfi.rx_antenna?"SMA":"PCB");
      LCD_DrawString(127,(i*9)+5, textbuf, COLOR_FILL, ALIGN_RIGHT);
      break;
    case 3:
      LCD_DrawString(10,(i*9)+5, "UART Mode", COLOR_FILL, ALIGN_LEFT);
      my_sprintf(textbuf, "%s", uart_mode?"TEXT":"SLIP");
      LCD_DrawString(127,(i*9)+5, textbuf, COLOR_FILL, ALIGN_RIGHT);
      break;
    }
  }
  
  if(edit)
  {
    LCD_FillRect(0,(state_s*9)+10,128,10, COLOR_INVERT);
    
    if(GetButton1())
    {
      GUI_DrawButtonL(label_dec, 1);
      switch(state_s)
      {
      case 0:
        switch(nbfi.tx_phy_channel)
        {
          case UL_DBPSK_50_PROT_D:
            break;
          case UL_DBPSK_400_PROT_D:
            nbfi.tx_phy_channel = UL_DBPSK_50_PROT_D;
            break;
          case UL_DBPSK_3200_PROT_D:
            nbfi.tx_phy_channel = UL_DBPSK_400_PROT_D;
            break;
        }
        //if(nbfi.tx_phy_channel > 0) nbfi.tx_pwr--;
        break;
      case 1:
        nbfi.tx_antenna ^= 1;
        break;
      case 2:
        nbfi.rx_antenna ^= 1;
        break;
      case 3:
        uart_mode ^= 1;
        break;
      }
    }
    else
    {
      GUI_DrawButtonL(label_dec, 0);
    }
    
    if(GetButton2())
    {
      GUI_DrawButtonR(label_inc, 1);
      switch(state_s)
      {
      case 0:
        switch(nbfi.tx_phy_channel)
        {
          case UL_DBPSK_50_PROT_D:
            nbfi.tx_phy_channel = UL_DBPSK_400_PROT_D;
            break;
          case UL_DBPSK_400_PROT_D:
            nbfi.tx_phy_channel = UL_DBPSK_3200_PROT_D;
            break;
          case UL_DBPSK_3200_PROT_D:
            break;
        }
        //if(nbfi.tx_pwr < 26) nbfi.tx_pwr++;
        break;
      case 1:
        nbfi.tx_antenna ^= 1;
        
        break;
      case 2:
        nbfi.rx_antenna ^= 1;
        break;
      case 3:
        uart_mode ^= 1;
        break;
        
      }
    }
    else
    {
      GUI_DrawButtonR(label_inc, 0);
    }
    
    if(GetButton3())
    {
      edit = 0;
      return;
    }
  }

  // Button processing
  if(edit == 0)
  {
    // Cursor
    LCD_DrawString(5,(state_s*9)+5,">", COLOR_FILL, ALIGN_LEFT);
    
    if(GetButton1())
    {
      GUI_DrawButtonL(label_next, 1);
      state_s++;
      state_s = state_s % 4;
    }
    else
    {
      GUI_DrawButtonL(label_next, 0);
    }
    
    if(GetButton2())
    {
      GUI_DrawButtonR(label_edit, 1);
      edit = 1;
    }
    else
    {
      GUI_DrawButtonR(label_edit, 0);
    }
    
    if(GetButton3())
    {
      state_s = 0;
      current_handler = &MainHandler;
    }
  }
}


const gui_entry_t info_table[]=
{
  {"NBFi quality",      &NBFiQuality},
  {"NBFi statistics",   &NBFiStats},
  {"Device",            &DevInfoHandler},
};

void InfoHandler()
{
  static uint8_t state_i = 0;
  
  LCD_DrawString(0,(uint16_t)-6,"../Info", COLOR_FILL, ALIGN_LEFT);
  
  // Entry list
  for(int i=0; i<TABLE_SIZE(info_table); i++)
  {
    LCD_DrawString(20,(i*9)+5,info_table[i].label, COLOR_FILL, ALIGN_LEFT);
  }
  
  // Cursor
  LCD_DrawString(10,(state_i*9)+5,">", COLOR_FILL, ALIGN_LEFT);
  
  // Button processing
  if(GetButton1())
  {
    GUI_DrawButtonL(label_next, 1);
    state_i++;
    state_i = state_i % TABLE_SIZE(info_table);
  }
  else
  {
    GUI_DrawButtonL(label_next, 0);
  }
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_enter, 1);
    current_handler = info_table[state_i].handler;
  }
  else
  {
    GUI_DrawButtonR(label_enter, 0);
  }
  
  if(GetButton3())
  {
    state_i = 0;
    current_handler = &MainHandler;
  }
}


extern int16_t noise;
void NBFiQuality()
{
  LCD_DrawString(0,(uint16_t)-6,"../Info/NBFi quality", COLOR_FILL, ALIGN_LEFT);
  
  LCD_DrawString(10,5,"Noise lev:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d dBm", noise);
  LCD_DrawString(127,5, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,15,"UL aver. SNR:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", nbfi_state.aver_tx_snr);
  LCD_DrawString(127,15, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,25,"DL aver. SNR:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", nbfi_state.aver_rx_snr);
  LCD_DrawString(127,25, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,35,"UL bitrate:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", NBFi_Phy_To_Bitrate(nbfi.tx_phy_channel));
  LCD_DrawString(127,35, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,45,"DL bitrate:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%u", NBFi_Phy_To_Bitrate(nbfi.rx_phy_channel));
  LCD_DrawString(127,45, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  if(GetButton3())
  {
    current_handler = &InfoHandler;
  }
}

void NBFiStats()
{
  LCD_DrawString(0,(uint16_t)-6,"../Info/NBFi statistics", COLOR_FILL, ALIGN_LEFT);
  
  LCD_DrawString(10,5,"UL enqueued:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", NBFi_Packets_To_Send());
  LCD_DrawString(127,5, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,15,"UL delivered:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", nbfi_state.success_total);
  LCD_DrawString(127,15, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,25,"UL lost:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", nbfi_state.fault_total);
  LCD_DrawString(127,25, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,35,"UL total:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", nbfi_state.UL_total);
  LCD_DrawString(127,35, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,45,"DL total:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", nbfi_state.DL_total);
  LCD_DrawString(127,45, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  if(GetButton3())
  {
    current_handler = &InfoHandler;
  }
}

extern RTC_HandleTypeDef hrtc;                                                  //added by me
extern uint32_t buttons_v;
extern uint32_t VDD;
void DevInfoHandler()
{
  RTC_TimeTypeDef time;                                                         //added by me
  RTC_DateTypeDef date;                                                         //added by me
  
  LCD_DrawString(0,(uint16_t)-6,"../Info/Device info", COLOR_FILL, ALIGN_LEFT);
  
  // Device full ID
  LCD_DrawString(10,5,"ID:", COLOR_FILL, ALIGN_LEFT);
  //    my_sprintf(textbuf, "ID: %02X%02X%02X%02X%02X%02X", nbfi.full_ID[5],nbfi.full_ID[4],nbfi.full_ID[3],nbfi.full_ID[2],nbfi.full_ID[1],nbfi.full_ID[0]);
  my_sprintf(textbuf, "%02X%02X%02X", nbfi.full_ID[1],nbfi.full_ID[2],nbfi.full_ID[3]);
  LCD_DrawString(127,5,textbuf, COLOR_FILL, ALIGN_RIGHT);
  
                                                                                //time_t t = RTC_Time();
                                                                                //struct tm *TM = localtime(&t);
  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);                                //added by me
  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);                                //added by me
  
  LCD_DrawString(10,15,"Time:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%02d:%02d:%02d", time.Hours, time.Minutes, time.Seconds);//my_sprintf(textbuf, "%02d:%02d:%02d", TM->tm_hour, TM->tm_min, TM->tm_sec);
  LCD_DrawString(127,15,textbuf, COLOR_FILL, ALIGN_RIGHT);

  
  //    LCD_DrawString(10,23,"Buttons:", COLOR_FILL, ALIGN_LEFT);
  //    my_sprintf(textbuf, "%u", buttons_v);
  //    LCD_DrawString(127,23,textbuf, COLOR_FILL, ALIGN_RIGHT);
  //
  LCD_DrawString(10,25,"VCC:", COLOR_FILL, ALIGN_LEFT);
                                                                                //my_sprintf(textbuf, "%umV", ((VDD*10000)>>16) - 4500);
  LCD_DrawString(127,25,textbuf, COLOR_FILL, ALIGN_RIGHT);
  //
  LCD_DrawString(10,35,"TEMP:", COLOR_FILL, ALIGN_LEFT);
                                                                                //my_sprintf(textbuf, "%d'C", adc_measure_temperature() >> 8);
  LCD_DrawString(127,35,textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_reflash, 1);
                                                                                //HAL_RebootToBootloader();
  }
  else
  {
    GUI_DrawButtonR(label_reflash, 0);
  }
  
  if(GetButton3())
  {
    current_handler = &InfoHandler;
  }
}

//-------------------------------- SERVICE ROUTINES -------------------------------------

void GUI_Update()
{
  LCD_ClearBuffer(0);
  LCD_SetFont(&arial_8ptFontInfo);
  
  (*current_handler)();
  
  LCD_WriteBuffer();
}

void GUI_Init()
{
  //ScheduleTask(&gui_handler, GUI_Handler, RELATIVE, 0);
  current_handler = &MainHandler;
}

void GUI_DrawButtonL(const char *label, uint8_t state)
{
  if(state)
  {
    LCD_FillRect(0, 52, 51, 12, 1);
    LCD_DrawString(26,48,label, COLOR_INVERT, ALIGN_CENTERED);
  }
  else
  {
    LCD_DrawRect(0, 52, 51, 12, 1);
    LCD_DrawString(26,48,label, COLOR_FILL, ALIGN_CENTERED);
  }
}

void GUI_DrawButtonR(const char *label, uint8_t state)
{
  if(state)
  {
    LCD_FillRect(76, 52, 51, 12, 1);
    LCD_DrawString(102,48,label, COLOR_INVERT, ALIGN_CENTERED);
  }
  else
  {
    LCD_DrawRect(76, 52, 51, 12, 1);
    LCD_DrawString(102,48,label, COLOR_FILL, ALIGN_CENTERED);
  }
}
