#ifndef NBFI_CONFIG_H
#define NBFI_CONFIG_H

#define READ_PARAM_CMD              0x00
#define WRITE_PARAM_CMD             0x01
#define WRITE_PARAM_WITH_ACK_CMD    0x02
#define WRITE_PARAM_AND_SAVE_CMD    0x03

#define NBFI_PARAM_MODE         0x00
#define NBFI_PARAM_HANDSHAKE    0x01
#define NBFI_PARAM_MAXLEN       0x02
#define NBFI_PARAM_TXFREQ       0x03
#define NBFI_PARAM_RXFREQ       0x04
#define NBFI_PARAM_ANT          0x05
#define NBFI_PARAM_DL_ADD       0x06
#define NBFI_PARAM_HEART_BEAT   0x07
#define NBFI_PARAM_TX_BRATES    0x08
#define NBFI_PARAM_RX_BRATES    0x09
#define NBFI_PARAM_VERSION      0x0A
#define NBFI_ADD_FLAGS          0x0B
#define NBFI_QUALITY            0x0C
#define NBFI_UL_BASE_FREQ       0x0D
#define NBFI_DL_BASE_FREQ       0x0E
#define NBFI_QUALITY_EX         0x0F

typedef struct
{
    nbfi_mode_t mode;
    nbfi_phy_channel_t tx_phy_channel;
    nbfi_phy_channel_t rx_phy_channel;
    nbfi_handshake_t   handshake_mode;
    nbfi_mack_mode_t   mack_mode;
    uint8_t     num_of_retries;
    uint8_t     max_payload_len;
    uint8_t     dl_ID[3];
    uint8_t     temp_ID[3];
    uint8_t     broadcast_ID[3];
    uint8_t     full_ID[6];
    uint32_t    tx_freq;
    uint32_t    rx_freq;
    uint8_t     tx_antenna;
    uint8_t     rx_antenna;
    int8_t      tx_pwr;
    uint16_t    heartbeat_interval;
    uint8_t     heartbeat_num;
    uint8_t     additional_flags;
    uint32_t    ul_freq_base;
    uint32_t    dl_freq_base;
    uint8_t     reserved[3];
}nbfi_settings_t;

extern nbfi_settings_t nbfi;

//aditional flags:
#define NBFI_FLG_FIXED_BAUD_RATE        0x01
#define NBFI_FLG_NO_RESET_TO_DEFAULTS   0x02
#define NBFI_FLG_NO_SENDINFO            0x04
#define NBFI_FLG_NO_XTEA                0x08
#define NBFI_FLG_DO_OSCCAL              0x10
#define NBFI_FLG_NO_REDUCE_TX_PWR       0x20
#define NBFI_OFF_MODE_ON_INIT           0x40

typedef struct
{
        uint32_t modem_id;
        uint32_t* key;
	uint8_t tx_min_pwr;
	uint8_t tx_max_pwr;
	uint8_t hardware_id;
	uint8_t hardware_rev;
	uint8_t band_id;
	uint32_t send_info_interval;
}nbfi_dev_info_t;
extern nbfi_dev_info_t dev_info;

//BAND IDs
#define UL868800_DL446000            0
#define UL868800_DL864000            1
#define UL868800_DL446000_DL864000   2
#define UL867950_DL446000            3
#define UL868500_DL446000            4
#define UL868100_DL446000            5
#define UL864000_DL446000            6
#define UL863175_DL446000            7
#define UL864000_DL875000            8

typedef enum
{
    DOWN = 0,     // data rate change down direction
    UP = 1        // data rate change up direction
}nbfi_rate_direct_t;


typedef union
{
        struct
        {
            uint8_t RTC_MSB          : 6;//LSB
            uint8_t DL_SPEED_NOT_MAX : 1;
            uint8_t UL_SPEED_NOT_MAX : 1;
        };
        uint8_t info;
}NBFi_station_info_s;

 

extern NBFi_station_info_s nbfi_station_info;

void NBFi_Config_Set_Device_Info(nbfi_dev_info_t *);
nbfi_settings_t* NBFi_get_settings();
_Bool NBFi_Config_Parser(uint8_t* buf);
void NBFi_Clear_Saved_Configuration();
void NBFi_Config_Set_FastDl(_Bool);
_Bool NBFi_Is_Mode_Normal();

#endif // NBFI_CONFIG_H
