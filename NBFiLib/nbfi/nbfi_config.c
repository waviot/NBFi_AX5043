#include "nbfi.h"
#include "nbfi_config.h"
#include "nbfi_phy.h"
#include "nbfi_misc.h"
#include "rf.h"
#include <wtimer.h>
#include <stdlib.h>
#include <string.h>
#include "nbfi_defines.h"
#include "XTEA.h"


#define memset_xdata memset
#define memcpy_xdata memcpy
#define memcpy_xdatageneric memcpy
#define memcpy_genericxdata memcpy

#define RX_CONF 0x01
#define TX_CONF 0x02

_Bool NBFi_Config_Tx_Idle();

nbfi_settings_t nbfi;


nbfi_dev_info_t dev_info =
{
    0x700000,
    0,
    RF_MIN_POWER,
    RF_MAX_POWER,
    HARDWARE_ID,
    HARDWARE_REV,
    BAND_ID,
    SEND_INFO_INTERVAL
};


const nbfi_settings_t nbfi_fastdl =
{
    CRX,
    UL_PSK_FASTDL,  
    DL_PSK_FASTDL,  
    HANDSHAKE_NONE,
    MACK_1, 
    1,
    128,
    {0xFF,0,0},   
    {0},    
    {0xFF,0,0},     
    {0},    
    868800000,      
    865000000,      
    PCB,    
    PCB,    
    RF_MAX_POWER,     
    60,     
    0,      
    NBFI_FLG_FIXED_BAUD_RATE | NBFI_FLG_NO_RESET_TO_DEFAULTS | NBFI_FLG_NO_SENDINFO |NBFI_FLG_NO_XTEA,
    0,
    0
};


NBFi_station_info_s nbfi_station_info = {0};

extern uint8_t  string[50];

nbfi_settings_t nbfi_prev;

_Bool nbfi_settings_changed = 0;


#define NUM_OF_TX_RATES    3
#define NUM_OF_RX_RATES    4

nbfi_phy_channel_t TxRateTable[NUM_OF_TX_RATES] = {UL_DBPSK_50_PROT_D, UL_DBPSK_400_PROT_D, UL_DBPSK_3200_PROT_D};
const uint8_t TxSNRDegradationTable[NUM_OF_TX_RATES] = {0,9,18};
nbfi_phy_channel_t RxRateTable[NUM_OF_RX_RATES] = {DL_PSK_200, DL_PSK_500, DL_PSK_5000, DL_PSK_FASTDL};
const uint8_t RxSNRDegradationTable[NUM_OF_RX_RATES] = {0,4,14,25};

#define TX_SNRLEVEL_FOR_UP          15//22
#define TX_SNRLEVEL_FOR_DOWN        10

#define RX_SNRLEVEL_FOR_UP          15//18
#define RX_SNRLEVEL_FOR_DOWN        10


uint8_t current_tx_rate = 0;
uint8_t current_rx_rate = 0;

uint8_t prev_tx_rate = 0;
uint8_t prev_rx_rate = 0;

uint8_t success_rate = 0;

uint8_t you_should_dl_power_step_up = 0;
uint8_t you_should_dl_power_step_down = 0;

extern nbfi_state_t nbfi_state;
extern nbfi_transport_packet_t* nbfi_active_pkt;

static _Bool NBFi_Config_Rate_Change(uint8_t rx_tx, nbfi_rate_direct_t dir );
_Bool NBFi_Config_Tx_Power_Change(nbfi_rate_direct_t dir);
void NBFi_Config_Return();
_Bool NBFi_Config_Send_Mode(_Bool, uint8_t);
void NBFi_Config_Set_Default();
void NBFi_ReadConfig();
void NBFi_WriteConfig();
void NBFi_Config_Set_TX_Chan(nbfi_phy_channel_t ch);
void NBFi_Config_Set_RX_Chan(nbfi_phy_channel_t ch);
void NBFi_Config_Send_Current_Mode(struct wtimer_desc *desc);


static _Bool NBFI_Config_is_high_SNR_for_UP(uint8_t rx_tx)
{
    uint8_t delta;
    if(rx_tx & RX_CONF)
    {
        if(current_rx_rate < (NUM_OF_RX_RATES - 1))
        {
            delta = RxSNRDegradationTable[current_rx_rate + 1] - RxSNRDegradationTable[current_rx_rate];
            if(nbfi_state.aver_rx_snr  > RX_SNRLEVEL_FOR_UP + delta) return 1;
            else return 0;
        }
        else
        {
            if(nbfi_state.aver_rx_snr  > RX_SNRLEVEL_FOR_UP + 10) return 1;
            else return 0;
        }


    }

    if(rx_tx & TX_CONF)
    {
        if(current_tx_rate < (NUM_OF_TX_RATES - 1))
        {
            delta = TxSNRDegradationTable[current_tx_rate + 1] - TxSNRDegradationTable[current_tx_rate];
            if(nbfi_state.aver_tx_snr  > TX_SNRLEVEL_FOR_UP + delta) return 1;
            else return 0;
        }
        else
        {
            if(nbfi_state.aver_tx_snr  > TX_SNRLEVEL_FOR_UP + 10) return 1;
            else return 0;
        }
    }
    return 0;
}


void NBFI_Config_Check_State()
{
    if(nbfi.tx_phy_channel != UL_PSK_FASTDL)
    {
        nbfi_state.UL_rating = (nbfi_state.aver_tx_snr + TxSNRDegradationTable[current_tx_rate]);
        if(nbfi_state.UL_rating > 40) nbfi_state.UL_rating = 40;
        nbfi_state.UL_rating >>= 2;

        nbfi_state.DL_rating = (nbfi_state.aver_rx_snr + RxSNRDegradationTable[current_rx_rate]);
        if(nbfi_state.DL_rating > 40) nbfi_state.DL_rating = 40;
        nbfi_state.DL_rating >>= 2;
    }


    #ifdef FIXED_BAUD_RATE
    return;
    #endif // FIXED_BAUD_RATE
    if(nbfi.mode == NRX) return;
    if(nbfi.additional_flags&NBFI_FLG_FIXED_BAUD_RATE) return;
    if(nbfi.handshake_mode == HANDSHAKE_NONE) return;
    switch(nbfi_active_pkt->state)
    {
        case PACKET_WAIT_ACK:
        case PACKET_WAIT_FOR_EXTRA_PACKETS:
        return;
    }
    
    if(you_should_dl_power_step_down && (nbfi_state.aver_rx_snr < RX_SNRLEVEL_FOR_UP)) you_should_dl_power_step_down = 0;
    if(you_should_dl_power_step_up && (nbfi_state.aver_rx_snr > RX_SNRLEVEL_FOR_DOWN)) you_should_dl_power_step_up = 0;

     if(NBFI_Config_is_high_SNR_for_UP(TX_CONF)&& NBFI_Config_is_high_SNR_for_UP(RX_CONF))
    {
        if(!NBFi_Config_Rate_Change(RX_CONF|TX_CONF, UP))
        {
            NBFi_Config_Tx_Power_Change(DOWN);
            you_should_dl_power_step_down = (1 << 7);
        }

        return;
    }

    if(NBFI_Config_is_high_SNR_for_UP(TX_CONF))
    {
        if(!NBFi_Config_Rate_Change(TX_CONF, UP)) NBFi_Config_Tx_Power_Change(DOWN);
        return;
    }

    if(NBFI_Config_is_high_SNR_for_UP(RX_CONF))
    {
        if(!NBFi_Config_Rate_Change(RX_CONF, UP)) you_should_dl_power_step_down = (1 << 7);
        return;
    }

    if(nbfi_state.aver_tx_snr < TX_SNRLEVEL_FOR_DOWN)
    {
        if(!NBFi_Config_Tx_Power_Change(UP)) NBFi_Config_Rate_Change(TX_CONF, DOWN);
    }

    if(nbfi_state.aver_rx_snr < RX_SNRLEVEL_FOR_DOWN)
    {
        you_should_dl_power_step_up = (1 << 6);
        NBFi_Config_Rate_Change(RX_CONF, DOWN);
        return;
    }
}

static _Bool NBFi_Config_Rate_Change(uint8_t rx_tx, nbfi_rate_direct_t dir )
{
    uint8_t  rx = current_rx_rate;
    uint8_t  tx = current_tx_rate;
    uint8_t should_not_to_reduce_pwr = 0;
    if((rx_tx & RX_CONF) && NBFi_Config_Tx_Idle())
    {
        if((dir == UP)&& nbfi_station_info.DL_SPEED_NOT_MAX)
        {
            if(++current_rx_rate > NUM_OF_RX_RATES - 1)  current_rx_rate = NUM_OF_RX_RATES - 1;
        }
        if(dir == DOWN)
        {
            if(((int8_t)(--current_rx_rate)) < 0 ) current_rx_rate = 0;
        }
    }

    if((rx_tx & TX_CONF))
    {
        if((dir == UP)&& nbfi_station_info.UL_SPEED_NOT_MAX)
        {
            if(++current_tx_rate > NUM_OF_TX_RATES - 1)  current_tx_rate = NUM_OF_TX_RATES - 1;
            if(RxRateTable[current_rx_rate] == DL_PSK_200) 
              while(TxRateTable[current_tx_rate] != UL_DBPSK_50_PROT_D)
              {
                current_tx_rate--;
                should_not_to_reduce_pwr = 1;
              }
            if(RxRateTable[current_rx_rate] == DL_PSK_500) 
              while(TxRateTable[current_tx_rate] != UL_DBPSK_400_PROT_D)
              {
                current_tx_rate--;
                should_not_to_reduce_pwr = 1;
              }
        }
        if(dir == DOWN)
        {
            if(((int8_t)(--current_tx_rate)) < 0 ) current_tx_rate = 0;
        }
    }
    if((nbfi.tx_phy_channel == TxRateTable[current_tx_rate]) && (nbfi.rx_phy_channel == RxRateTable[current_rx_rate]))
    {
        if(should_not_to_reduce_pwr) return 1;
        else return 0;
    }

    memcpy_xdata(&nbfi_prev, &nbfi, sizeof(nbfi));
    prev_rx_rate = rx;
    prev_tx_rate = tx;

    nbfi.tx_phy_channel = TxRateTable[current_tx_rate];

    if((nbfi.rx_phy_channel == RxRateTable[current_rx_rate]) && (dir == DOWN)) return 1;

    nbfi.rx_phy_channel = RxRateTable[current_rx_rate];

    if(!NBFi_Config_Send_Mode(1, NBFI_PARAM_MODE))
    {
        NBFi_Config_Return();
        return 0;
    }
    if(rx < current_rx_rate) nbfi_state.aver_rx_snr -= 10;
    if(tx < current_tx_rate) nbfi_state.aver_tx_snr -= 10;

    return 1;
}

_Bool NBFi_Config_Tx_Power_Change(nbfi_rate_direct_t dir)
{
    if(nbfi.additional_flags&NBFI_FLG_NO_REDUCE_TX_PWR) return 0;

    int8_t old_pwr = nbfi.tx_pwr;
    if(dir == UP)
    {
        nbfi.tx_pwr += 3;
        nbfi_state.aver_tx_snr += 3;
        if(nbfi.tx_pwr > dev_info.tx_max_pwr) nbfi.tx_pwr = dev_info.tx_max_pwr;
    }
    else
    {
        nbfi.tx_pwr -= 3;
        nbfi_state.aver_tx_snr -= 3;
        if(nbfi.tx_pwr < dev_info.tx_min_pwr) nbfi.tx_pwr = dev_info.tx_min_pwr;
    }
    return (nbfi.tx_pwr != old_pwr);
}


void NBFi_Config_Send_Current_Mode(struct wtimer_desc *desc)
{
    NBFi_Config_Send_Mode(0, NBFI_PARAM_MODE);
    NBFi_Send_Clear_Cmd(0);
}

_Bool NBFi_Config_Send_Mode(_Bool ack, uint8_t param)
{
    nbfi_transport_packet_t* ack_pkt =  NBFi_AllocateTxPkt(8);

    if(!ack_pkt)
    {
        return 0;
    }
    ack_pkt->phy_data.payload[0] = 0x06;
    ack_pkt->phy_data.payload[1] = (READ_PARAM_CMD << 6) + param;
    NBFi_Config_Parser(&ack_pkt->phy_data.payload[1]);
    ack_pkt->phy_data.ITER = nbfi_state.UL_iter & 0x1f;;
    ack_pkt->phy_data.header |= SYS_FLAG;
    if(ack)
    {
        ack_pkt->handshake = nbfi.handshake_mode;
        ack_pkt->phy_data.header |= ACK_FLAG;
    }
    ack_pkt->state = PACKET_NEED_TO_SEND_RIGHT_NOW;
    return 1;
}

void bigendian_cpy(uint8_t* from, uint8_t* to, uint8_t len)
{
    for(uint8_t i = 0; i != len; i++)
    {
        to[i] = from[len - i - 1];
    }
}

uint8_t CompVersion()
{

    const char CompTime[] = __TIME__;
    const char* ptr;
    uint8_t ver = 0;
    ptr = &CompTime[0];
    while(*ptr) ver += ((*(ptr++)) - 0x30);
    return ver;
}

_Bool NBFi_Config_Parser(uint8_t* buf)
{
    switch(buf[0]>>6)
    {
        case READ_PARAM_CMD:
                memset_xdata(&buf[1], 0xff, 6);
                switch(buf[0]&0x3f)
                {
                    case NBFI_PARAM_MODE:
                        buf[1] = nbfi.mode;
                        buf[2] = nbfi.mack_mode;
                        buf[3] = nbfi.tx_phy_channel;
                        buf[4] = nbfi.rx_phy_channel;
                        buf[5] = nbfi.tx_pwr;
                        buf[6] = nbfi.num_of_retries;
                        break;
                    case NBFI_PARAM_HANDSHAKE:
                        buf[1] = nbfi.handshake_mode;
                        buf[2] = nbfi.mack_mode;
                        break;
                    case NBFI_PARAM_MAXLEN:
                        buf[1] = nbfi.max_payload_len;
                        break;
                    case NBFI_PARAM_TXFREQ:
                        bigendian_cpy((uint8_t*)&nbfi.tx_freq, &buf[1], 4);
                        break;
                    case NBFI_PARAM_RXFREQ:
                        bigendian_cpy((uint8_t*)&nbfi.rx_freq, &buf[1], 4);
                        break;
                    case NBFI_PARAM_ANT:
                        buf[1] = nbfi.tx_pwr;
                        buf[2] = nbfi.tx_antenna;
                        buf[3] = nbfi.rx_antenna;
                        break;
                    case NBFI_PARAM_DL_ADD:
                        for(uint8_t i = 0; i != 3; i++)  buf[1 + i] = nbfi.dl_ID[i];
                        break;
                    case NBFI_PARAM_HEART_BEAT:
                        buf[1] = nbfi.heartbeat_num;
                        buf[2] = nbfi.heartbeat_interval >> 8;
                        buf[3] = nbfi.heartbeat_interval & 0xff;
                        break;
                    case NBFI_PARAM_TX_BRATES:
                        for(uint8_t i = 0; i != NUM_OF_TX_RATES; i++)
                        {
                            if(i > 5) break;
                            buf[i + 1] = TxRateTable[i];
                        }
                        break;
                    case NBFI_PARAM_RX_BRATES:
                        for(uint8_t i = 0; i != NUM_OF_RX_RATES; i++)
                        {
                            if(i > 5) break;
                            buf[i + 1] = RxRateTable[i];
                        }
                        break;
                    case NBFI_PARAM_VERSION:
                        buf[1] = NBFI_REV;
                        buf[2] = NBFI_SUBREV;
                        buf[3] = CompVersion();
                        buf[4] = dev_info.hardware_id;
                        buf[5] = dev_info.hardware_rev;
                        buf[6] = dev_info.band_id;
                        break;
                    case NBFI_QUALITY:
                        bigendian_cpy((uint8_t*)&nbfi_state.UL_total, &buf[1], 2);
                        bigendian_cpy((uint8_t*)&nbfi_state.DL_total, &buf[3], 2);
                        buf[5] = nbfi_state.aver_rx_snr;
                        buf[6] = nbfi_state.aver_tx_snr;
                        break;
                    case NBFI_QUALITY_EX :
                        buf[1] = nbfi_state.UL_rating;
                        buf[2] = nbfi_state.DL_rating;
                        bigendian_cpy((uint8_t*)&nbfi_state.success_total, &buf[3], 2);
                        bigendian_cpy((uint8_t*)&nbfi_state.fault_total, &buf[5], 2);
                        break;
                    case NBFI_ADD_FLAGS:
                        buf[1] = nbfi.additional_flags;
                        break;
                    case NBFI_UL_BASE_FREQ:
                        bigendian_cpy((uint8_t*)&nbfi.ul_freq_base, &buf[1], 4);
                        break;
                    case NBFI_DL_BASE_FREQ:
                        bigendian_cpy((uint8_t*)&nbfi.dl_freq_base, &buf[1], 4);
                        break;
                    default:
                        return 0;
                        break;
                }
            break;

            case WRITE_PARAM_AND_SAVE_CMD:
                NBFi_Config_Set_Default();
            case WRITE_PARAM_CMD:
                memcpy_xdata(&nbfi_prev, &nbfi, sizeof(nbfi));
                switch(buf[0]&0x3f)
                {
                    case NBFI_PARAM_MODE:
                        if(buf[1] != 0xff) nbfi.mode = (nbfi_mode_t)buf[1];
                        if(buf[2] != 0xff) nbfi.mack_mode = (nbfi_mack_mode_t)buf[2];
                        if(buf[3] != 0xff) NBFi_Config_Set_TX_Chan((nbfi_phy_channel_t)buf[3]);
                        if(buf[4] != 0xff) {NBFi_Config_Set_RX_Chan((nbfi_phy_channel_t)buf[4]); rf_state = STATE_CHANGED;}
                        if(buf[5] != 0xff) nbfi.tx_pwr = buf[5];
                        if(buf[6] != 0xff) nbfi.num_of_retries = buf[6];
                        break;
                    case NBFI_PARAM_HANDSHAKE:
                        if(buf[1] != 0xff)
                        {
                            nbfi.handshake_mode = (nbfi_handshake_t)buf[1];
                        }
                        if(buf[2] != 0xff) nbfi.mack_mode = (nbfi_mack_mode_t)buf[2];
                        break;
                    case NBFI_PARAM_MAXLEN:
                        nbfi.max_payload_len = buf[1];
                        break;
                    case NBFI_PARAM_TXFREQ:
                        bigendian_cpy(&buf[1], (uint8_t*)&nbfi.tx_freq, 4);
                        if(buf[5] != 0xff) nbfi.tx_pwr = buf[5];
                        if(buf[6] != 0xff) nbfi.tx_antenna = buf[6];
                        break;
                    case NBFI_PARAM_RXFREQ:
                        bigendian_cpy(&buf[1], (uint8_t*)&nbfi.rx_freq, 4);
                        rf_state = STATE_CHANGED;
                        break;
                    case NBFI_PARAM_ANT:
                        if(buf[1] != 0xff) nbfi.tx_pwr = buf[1];
                        if(buf[2] != 0xff) nbfi.tx_antenna = buf[2];
                        if(buf[3] != 0xff) {nbfi.rx_antenna = buf[3]; rf_state = STATE_CHANGED;}
                        break;
                    case NBFI_PARAM_DL_ADD:
                        for(uint8_t i = 0; i != 3; i++)  nbfi.dl_ID[i] = buf[1 + i];
                        break;
                    case NBFI_PARAM_HEART_BEAT:
                        nbfi.heartbeat_num = buf[1];
                        nbfi.heartbeat_interval  = buf[2];
                        nbfi.heartbeat_interval <<= 8;
                        nbfi.heartbeat_interval += buf[3];
                        break;
                    case NBFI_ADD_FLAGS:
                        nbfi.additional_flags = buf[1];
                        break;
                    case NBFI_UL_BASE_FREQ:
                        bigendian_cpy(&buf[1], (uint8_t*)&nbfi.ul_freq_base, 4);
                        break;
                    case NBFI_DL_BASE_FREQ:
                        bigendian_cpy(&buf[1], (uint8_t*)&nbfi.dl_freq_base, 4);
                        break;
                    default:
                        nbfi_settings_changed = 0;
                        return 0;
                        break;
                }
                if(buf[0]>>6 == WRITE_PARAM_AND_SAVE_CMD)
                {
                    NBFi_WriteConfig();
                    NBFi_Config_Send_Mode(0, NBFI_PARAM_MODE);
                    return 0;
                }
            break;

    }
    buf[0] |= 0x80;
    return 1;
}


void NBFi_Config_Return()
{
    memcpy_xdata(&nbfi, &nbfi_prev, sizeof(nbfi));
    current_tx_rate = prev_tx_rate;
    current_rx_rate = prev_rx_rate;
    if(nbfi.mode == NRX) nbfi.handshake_mode = HANDSHAKE_NONE;
    NBFi_Config_Send_Mode(0, NBFI_PARAM_MODE);
}

void NBFi_Configure_IDs()
{
    nbfi.full_ID[0] = 0;
    nbfi.full_ID[1] = FULL_ID[2];
    nbfi.full_ID[2] = FULL_ID[1];
    nbfi.full_ID[3] = FULL_ID[0];
    nbfi.full_ID[4] = 0;
    nbfi.full_ID[5] = 0;
    nbfi.temp_ID[0] = nbfi.full_ID[1];
    nbfi.temp_ID[1] = nbfi.full_ID[2];
    nbfi.temp_ID[2] = nbfi.full_ID[3];

}

void NBFi_Config_Set_Default()
{

    NBFi_ReadConfig();

    NBFi_Configure_IDs();

    for(uint8_t i = 0; i != 3; i++) nbfi.dl_ID[i] = nbfi.temp_ID[i];   //default DL address

    srand(nbfi.full_ID[0] ^ nbfi.full_ID[1] ^ nbfi.full_ID[2] ^ nbfi.full_ID[3] ^ nbfi.full_ID[4] ^ nbfi.full_ID[5]);


    nbfi_state.aver_tx_snr = nbfi_state.aver_rx_snr = 0;
    current_tx_rate = current_rx_rate = 0;

    if(nbfi_active_pkt->state == PACKET_WAIT_ACK) nbfi_active_pkt->state = PACKET_LOST;

}

void NBFi_Config_Set_FastDl(_Bool fast)
{
    if(fast)
    {
        NBFi_Clear_TX_Buffer();
        uint32_t dl_base_freq = nbfi.dl_freq_base;
        for(uint8_t i = 0; i != sizeof(nbfi_settings_t); i++)
        {
            ((uint8_t*)&nbfi)[i] = ((uint8_t*)&nbfi_fastdl)[i];
        }
        nbfi.tx_freq = nbfi.rx_freq = dl_base_freq + 1000000;
        NBFi_Configure_IDs();
        for(uint8_t i = 0; i != 3; i++) nbfi.dl_ID[i] = nbfi.temp_ID[i];   //default DL address

    }
    else
    {
        NBFi_Config_Set_Default();
    }

    if(rf_state == STATE_RX) NBFi_RX();
}


_Bool NBFi_Config_Tx_Idle()
{
        switch(nbfi_active_pkt->state)
        {
        case PACKET_WAIT_ACK:
        case PACKET_WAIT_FOR_EXTRA_PACKETS:
        case PACKET_QUEUED_AGAIN:
            return 0;
        default:
            return 1;
        }
}


extern void (* __nbfi_read_flash_settings)(nbfi_settings_t*);
extern void (* __nbfi_read_default_settings)(nbfi_settings_t*);
void NBFi_ReadConfig()
{
	if(__nbfi_read_flash_settings == 0) goto read_default;

	__nbfi_read_flash_settings(&nbfi);

	if((nbfi.tx_phy_channel != 0xff) && (nbfi.tx_phy_channel != 0)) return;

read_default:

	if(__nbfi_read_default_settings) __nbfi_read_default_settings(&nbfi);

}

extern void (* __nbfi_write_flash_settings)(nbfi_settings_t*);
void NBFi_WriteConfig()
{
	if(__nbfi_write_flash_settings == 0) return;
	__nbfi_write_flash_settings(&nbfi);
}

void NBFi_Clear_Saved_Configuration()
{
  if(__nbfi_write_flash_settings == 0) return;
  nbfi_settings_t empty;
  empty.tx_phy_channel = DL_PSK_200;
  __nbfi_write_flash_settings(&empty);
}


void NBFi_Config_Set_TX_Chan(nbfi_phy_channel_t ch)
{
    #ifndef FIXED_BAUD_RATE
    uint8_t i;
    if(nbfi.additional_flags&NBFI_FLG_FIXED_BAUD_RATE) {nbfi.tx_phy_channel = ch; return;}
    for(i = 0; i != NUM_OF_TX_RATES; i++) if(TxRateTable[i] == ch) break;
    if(i == NUM_OF_TX_RATES) return;
    nbfi.tx_phy_channel = ch;
    if(current_tx_rate != i)
    {
        current_tx_rate = i;
        nbfi_state.aver_tx_snr = 15;
    }
    #else
    nbfi.tx_phy_channel = ch;
    #endif
}

void NBFi_Config_Set_RX_Chan(nbfi_phy_channel_t ch)
{
    #ifndef FIXED_BAUD_RATE
    uint8_t i;
    if(nbfi.additional_flags&NBFI_FLG_FIXED_BAUD_RATE) {nbfi.rx_phy_channel = ch; return;}
    for(i = 0; i != NUM_OF_RX_RATES; i++) if(RxRateTable[i] == ch) break;
    if(i == NUM_OF_RX_RATES) return;
    nbfi.rx_phy_channel = ch;
    if(current_rx_rate != i)
    {
        current_rx_rate = i;
        nbfi_state.aver_rx_snr = 15;
    }
    #else
    nbfi.rx_phy_channel = ch;
    #endif
}

_Bool NBFi_Is_Mode_Normal()
{
    return (nbfi.tx_phy_channel != UL_PSK_FASTDL);
}


void NBFi_Config_Set_Device_Info(nbfi_dev_info_t *info)
{
	dev_info = *info;
        if(dev_info.key) XTEA_Set_KEY(dev_info.key);
}

nbfi_settings_t* NBFi_get_settings()
{
    return &nbfi;
}

