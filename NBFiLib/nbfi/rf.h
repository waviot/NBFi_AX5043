#ifndef RF_H
#define RF_H

#include "nbfi.h"

#define PA_DIFFERENTIAL (0x1)
#define PA_SINGLE_ENDED (0x2)
#define PA_SHAPING      (0x4)

typedef enum
{
    STATE_OFF,
    STATE_RX,
    STATE_TX,
    STATE_CHANGED
}nbfi_rf_state_s;

typedef enum
{
    NO_PADDING = 0,
    PADDING_4TO1
}rf_padding_t;

typedef enum
{
    NONBLOCKING,
    BLOCKING,
}rf_blocking_t;


typedef struct
{
  uint8_t sysclk;
  uint8_t dclk;
  uint8_t data;
  uint8_t antsel;
  uint8_t cfga;
  uint16_t txpwr;
} NBFi_ax5043_pins_s;

typedef enum
{
    PSK_446_DBPSK_868 = 0,
    PSK_864 = 1,
    DBPSK_434 = 2
}NBFi_radio_bands_t;

extern  struct axradio_address fastdladdress;
extern _Bool send_auto_corr;
extern uint8_t PSK_BAND;
extern uint8_t DBPSK_BAND;
extern nbfi_rf_state_s rf_state;
extern nbfi_phy_channel_t nbfi_phy_channel;
extern _Bool rf_busy;
extern _Bool transmit;

void axradio_statuschange(struct axradio_status  *st);

void            RF_SetFreq(uint32_t freq);
void            RF_SetModeAndPower(int8_t dBm, rf_direction_t mode, rf_antenna_t ant);
void            RF_SetDstAddress(uint8_t * addr);
void            RF_SetLocalAddress(uint8_t * addr);
nbfi_status_t   RF_Init(nbfi_phy_channel_t  phy_channel,
                        rf_antenna_t        antenna,
                        int8_t              power,
                        uint32_t            freq);
nbfi_status_t   RF_Deinit();
nbfi_status_t   RF_Transmit(uint8_t* pkt, uint8_t len,  rf_padding_t padding, rf_blocking_t blocking);


#endif // NBFI_H
