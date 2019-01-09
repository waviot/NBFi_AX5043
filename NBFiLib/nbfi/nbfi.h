#ifndef NBFI_H
#define NBFI_H

#include "easyax5043.h"
#include "ax5043.h"

#define FULL_ID     ((uint8_t*)(&dev_info.modem_id))

#define NBFI_TX_PKTBUF_SIZE     64
#define NBFI_RX_PKTBUF_SIZE     32

typedef enum
{   
    NRX         =   0,
    DRX         =   1,
    CRX         =   2,
    TRANSPARENT =   3,
    OFF         =   4
}nbfi_mode_t;

typedef enum
{
    MACK_0      = 0,//no ack
    MACK_1      = 1,//0x1,
    MACK_2      = 2,//0x3,
    MACK_4      = 4,//0x0f,
    MACK_8      = 8,//0xff,
    MACK_16     = 16,//0xffff,
    MACK_32     = 32,//0,
}nbfi_mack_mode_t;

typedef enum
{
        DL_PSK_200              = 0,
        DL_PSK_500              = 1,
        DL_PSK_5000             = 2,
        DL_PSK_FASTDL           = 3,
	DL_DBPSK_50_PROT_D      = 10,
	DL_DBPSK_400_PROT_D	= 11,
	DL_DBPSK_3200_PROT_D	= 12,
	DL_DBPSK_25600_PROT_D	= 13,
        DL_DBPSK_50_PROT_E	= 14,
        DL_DBPSK_400_PROT_E	= 15,
        DL_DBPSK_3200_PROT_E	= 16,
        DL_DBPSK_25600_PROT_E	= 17,
        UL_DBPSK_50_PROT_C      = 20,
        UL_DBPSK_50_PROT_D      = 21,
        UL_PSK_200              = 22,
        UL_DBPSK_400_PROT_C     = 23,
        UL_DBPSK_400_PROT_D     = 24,
        UL_PSK_500              = 25,
        UL_DBPSK_3200_PROT_D    = 26,
        UL_PSK_5000             = 27,
        UL_DBPSK_25600_PROT_D   = 28,
        UL_PSK_FASTDL           = 29,
        UL_DBPSK_50_PROT_E	= 30,
        UL_DBPSK_400_PROT_E	= 31,
	UL_DBPSK_3200_PROT_E	= 32,
	UL_DBPSK_25600_PROT_E	= 33,
        UL_CARRIER              = 50,
        OSC_CAL                 = 51
}nbfi_phy_channel_t;


typedef enum
{
    HANDSHAKE_NONE      = 0,
    HANDSHAKE_SIMPLE    = 1
}nbfi_handshake_t;



typedef struct
{
    uint32_t UL_total;
    uint32_t UL_iter;
    uint32_t DL_total;
    uint32_t DL_iter;
    int16_t noise;
    uint8_t  aver_rx_snr;
    uint8_t  aver_tx_snr;
    int16_t success_total;
    int16_t fault_total;
    int16_t last_rssi;
    uint8_t UL_rating;
    uint8_t DL_rating;
    uint32_t DL_last_time;
    int16_t rssi;
}nbfi_state_t;


typedef enum
{
    OK = 0,
    ERR = 1,
    ERR_RF_BUSY = 2,
    ERR_ACK_LOST = 3,
    ERR_BUFFER_FULL = 4
}nbfi_status_t;

// NB-Fi header
#define SYS_FLAG        (1<<7)
#define ACK_FLAG        (1<<6)
#define MULTI_FLAG      (1<<5)


typedef enum
{
    PCB = 0,       //PCB or ANT 1
    SMA = 1        //SMA or ANT 2
}rf_antenna_t;

typedef enum
{
    RX = 0,
    TX = 1,
    IDLE = 2
}rf_direction_t;


typedef void (*rx_handler_t)(uint8_t*, uint16_t);

enum nbfi_func_t
{
    NBFI_ON_OFF_PWR,
	NBFI_BEFORE_TX,
	NBFI_BEFORE_RX,
    NBFI_BEFORE_OFF,
	NBFI_RECEIVE_COMLETE,
	NBFI_READ_DEFAULT_SETTINGS,
	NBFI_READ_FLASH_SETTINGS,
	NBFI_WRITE_FLASH_SETTINGS,
	NBFI_MEASURE_VOLTAGE_OR_TEMPERATURE,
    NBFI_UPDATE_RTC,
    NBFI_RTC_SYNCHRONIZED,
    NBFI_LOCKUNLOCKNBFIIRQ,
    NBFI_RESET
};

void 	        NBFI_reg_func(uint8_t name, void*);
nbfi_status_t   NBFI_Init();
void            NBFi_Go_To_Sleep(_Bool sleep);
nbfi_status_t   NBFi_Send(uint8_t* payload, uint8_t length);
void            NBFi_ProcessRxPackets(_Bool external);
uint8_t         NBFi_Packets_To_Send();
nbfi_state_t*   NBFi_get_state();
uint8_t         NBFi_can_sleep();
uint32_t        NBFi_get_RTC();
void            NBFi_set_RTC(uint32_t time);
#endif // NBFI_H
