#include <libmfwtimer.h>
#include "rf.h"
#include "nbfi_config.h"
#include "time.h"
#include "waviotdvk.h"
#include "gui.h"
#include "glcd.h"
#include "fonts.h"
#include "adc.h"
#include "main.h"

extern nbfi_state_t nbfi_state;
extern uint16_t NBFi_Phy_To_Bitrate(nbfi_phy_channel_t ch);
extern uint32_t nbfi_measure_valtage_or_temperature(uint8_t val);               //added
int16_t noise;
uint8_t *last_rx_pkt;
uint8_t last_rx_pkt_len;

struct wtimer_desc gui_handler;
void (*current_handler)(void);

char textbuf[30]; // for formatted strings

extern int my_sprintf(char *buf, char *fmt, ...);
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
const char label_enter[] =   "Enter";
const char label_back[] =    "Back";
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
    GUI_DrawButtonL(label_enter, 1);
    current_handler = main_table[state].handler;
  }
  else
  {
    GUI_DrawButtonL(label_enter, 0);
  }
  
//  if(GetButton2())
//  {
//    GUI_DrawButtonR(label_back, 1);
//    
//  }
//  else
//  {
//    GUI_DrawButtonR(label_back, 0);
//  }
  
  if(GetButton3())
  {
    state--;
    if(state > TABLE_SIZE(main_table))
      state = TABLE_SIZE(main_table)-1;
  }
  
  if(GetButton4())
  {
    state++;
    state = state % TABLE_SIZE(main_table);
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
    GUI_DrawButtonL(label_enter, 1);
    current_handler = packet_tests_table[state_h].handler;
  }
  else
  {
    GUI_DrawButtonL(label_enter, 0);
  }
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_back, 1);
    state_h = 0;
    current_handler = &MainHandler;
  }
  else
  {
    GUI_DrawButtonR(label_back, 0);
  }
  
  if(GetButton3())
  {
    state_h--;
    if(state_h > TABLE_SIZE(packet_tests_table))
      state_h = TABLE_SIZE(packet_tests_table)-1;
  }
  
  if(GetButton4())
  {
    state_h++;
    state_h = state_h % TABLE_SIZE(packet_tests_table);
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
    GUI_DrawButtonL(label_enter, 1);
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
    GUI_DrawButtonL(label_enter, 0);
  }
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_back, 1);
    state_n = 0;
    current_handler = &TestsHandler;
  }
  else
  {
    GUI_DrawButtonR(label_back, 0);
  }
  
  LCD_DrawString(10,36,"UL enqueued:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d", NBFi_Packets_To_Send());
  LCD_DrawString(127,36, textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  if(GetButton3())
  {
    state_n--;
    if(state_n > TABLE_SIZE(nbfi_tx_table))
      state_n = TABLE_SIZE(nbfi_tx_table)-1;
  }
  
  if(GetButton4())
  {
    state_n++;
    state_n = state_n % TABLE_SIZE(nbfi_tx_table);
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
  
  if(GetButton2())
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

    if(GetButton2())
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
    current_rssi=nbfi_state.rssi;
    my_sprintf(textbuf, "RSSi = %i dBm", current_rssi);
    LCD_DrawString(10,23,textbuf, COLOR_FILL, ALIGN_LEFT);
    break;
  }
  
  // Caption
  LCD_DrawString(0,(uint16_t)-6,"../Tests/RSSi", COLOR_FILL, ALIGN_LEFT);
  
  // Button processing
  if(state)
  {
    if(GetButton2())
    {
      GUI_DrawButtonR(label_stop, 1);
      RF_Deinit();
      state = 0;
    }
    else
    {
      GUI_DrawButtonR(label_stop, 0);
    }
  }
  else
  {
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
      GUI_DrawButtonR(label_back, 1);
      current_handler = &TestsHandler;
    }
    else
    {
      GUI_DrawButtonR(label_back, 0);
    }
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
      LCD_DrawString(10,(i*9)+5, " TX BitRate", COLOR_FILL, ALIGN_LEFT);
      my_sprintf(textbuf, "%d", NBFi_Phy_To_Bitrate(nbfi.tx_phy_channel));
      LCD_DrawString(127,(i*9)+5, textbuf, COLOR_FILL, ALIGN_RIGHT);
      break;
    case 1:
      LCD_DrawString(10,(i*9)+5, " TX Antenna", COLOR_FILL, ALIGN_LEFT);
      my_sprintf(textbuf, "%s", nbfi.tx_antenna?"SMA":"PCB");
      LCD_DrawString(127,(i*9)+5, textbuf, COLOR_FILL, ALIGN_RIGHT);
      break;
    case 2:
      LCD_DrawString(10,(i*9)+5, " RX Antenna", COLOR_FILL, ALIGN_LEFT);
      my_sprintf(textbuf, "%s", nbfi.rx_antenna?"SMA":"PCB");
      LCD_DrawString(127,(i*9)+5, textbuf, COLOR_FILL, ALIGN_RIGHT);
      break;
    case 3:
      LCD_DrawString(10,(i*9)+5, " NBFi Mode", COLOR_FILL, ALIGN_LEFT);
      if(nbfi.mode == DRX)
        my_sprintf(textbuf, "%s", "DRX");
      else
        my_sprintf(textbuf, "%s", nbfi.mode?"CRX":"NRX");
      LCD_DrawString(127,(i*9)+5, textbuf, COLOR_FILL, ALIGN_RIGHT);
      break;
    }
  }
  
  // Button processing
  if(edit)
  {
    LCD_FillRect(0,(state_s*9)+10,128,10, COLOR_INVERT);

    if(GetButton1())
    {
      GUI_DrawButtonL(label_inc, 1);
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
            nbfi.tx_phy_channel = UL_DBPSK_50_PROT_D;
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
        nbfi.mode++;
        nbfi.mode %= 3;
        break;
      }      
    }
    else
    {
      GUI_DrawButtonL(label_inc, 0);
    }
    
    if(GetButton2())
    {
      GUI_DrawButtonR(label_back, 1);
      edit = 0;
    }
    else
    {
      GUI_DrawButtonR(label_back, 0);
    }
  }
  else          //if (edit == 0);
  {
    // Cursor
    LCD_DrawString(5,(state_s*9)+5,">", COLOR_FILL, ALIGN_LEFT);
    
    if(GetButton1())
    {
      GUI_DrawButtonL(label_edit, 1);
      edit = 1;
    }
    else
    {
      GUI_DrawButtonL(label_edit, 0);
    }
    
    if(GetButton2())
    {
      GUI_DrawButtonR(label_back, 1);
      state_s = 0;
      current_handler = &MainHandler;
    }
    else
    {
      GUI_DrawButtonR(label_back, 0);
    }
    
    if(GetButton3())
    {
      state_s--;
      if(state_s > 4)
        state_s = 4-1;
    }
    
    if(GetButton4())
    {
      state_s++;
      state_s = state_s % 4;
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
    GUI_DrawButtonL(label_enter, 1);
    current_handler = info_table[state_i].handler;
  }
  else
  {
    GUI_DrawButtonL(label_enter, 0);
  }
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_back, 1);
    state_i = 0;
    current_handler = &MainHandler;
  }
  else
  {
    GUI_DrawButtonR(label_back, 0);
  }
  
  if(GetButton3())
  {
    state_i--;
    if(state_i > TABLE_SIZE(info_table))
      state_i = TABLE_SIZE(info_table)-1;
  }
  
  if(GetButton4())
  {
    state_i++;
    state_i = state_i % TABLE_SIZE(info_table);    
  }
}

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
  
  if(GetButton2())
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
  
  if(GetButton2())
  {
    current_handler = &InfoHandler;
  }
}

void DevInfoHandler()
{
  LCD_DrawString(0,(uint16_t)-6,"../Info/Device info", COLOR_FILL, ALIGN_LEFT);
  
  // Device full ID
  LCD_DrawString(10,5,"ID:", COLOR_FILL, ALIGN_LEFT);
  // my_sprintf(textbuf, "ID: %02X%02X%02X%02X%02X%02X", nbfi.full_ID[5],nbfi.full_ID[4],nbfi.full_ID[3],nbfi.full_ID[2],nbfi.full_ID[1],nbfi.full_ID[0]);
  my_sprintf(textbuf, "%02X%02X%02X", nbfi.full_ID[1],nbfi.full_ID[2],nbfi.full_ID[3]);
  LCD_DrawString(127,5,textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  time_t t = RTC_Time();
  struct tm *TM = localtime(&t);
  
  LCD_DrawString(10,15,"Time:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%02d:%02d:%02d", TM->tm_hour, TM->tm_min, TM->tm_sec);
  LCD_DrawString(127,15,textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  //    LCD_DrawString(10,23,"Buttons:", COLOR_FILL, ALIGN_LEFT);
  //    my_sprintf(textbuf, "%u", buttons_v);
  //    LCD_DrawString(127,23,textbuf, COLOR_FILL, ALIGN_RIGHT);
  
  LCD_DrawString(10,25,"VCC:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%umV", nbfi_measure_valtage_or_temperature(ADC_VDDA));
  LCD_DrawString(127,25,textbuf, COLOR_FILL, ALIGN_RIGHT);

  LCD_DrawString(10,35,"TEMP:", COLOR_FILL, ALIGN_LEFT);
  my_sprintf(textbuf, "%d'C", nbfi_measure_valtage_or_temperature(ADC_TEMP));
  LCD_DrawString(127,35,textbuf, COLOR_FILL, ALIGN_RIGHT);
  
//  if(GetButton1())
//  {
//    GUI_DrawButtonL(label_reflash, 1);
//    HAL_RebootToBootloader();
//  }
//  else
//  {
//    GUI_DrawButtonL(label_reflash, 0);
//  }
  
  if(GetButton2())
  {
    GUI_DrawButtonR(label_back, 1);
    current_handler = &InfoHandler;
  }
  else
  {
    GUI_DrawButtonR(label_back, 0);
  }
}

//-------------------------------- SERVICE ROUTINES -------------------------------------

void GUI_Update()
{
  LCD_ClearBuffer(0);
  LCD_SetFont(&arial_8ptFontInfo);
  
  (*current_handler)();
  
  ResetButtonFlags(0xff);                                                       //сброс всех флагов кнопок
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