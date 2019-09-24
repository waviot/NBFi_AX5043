#include "nbfi.h"
#include "nbfi_config.h"
#include "rf.h"
#include "nbfi_misc.h"
#include <libmfwtimer.h>

_Bool rf_busy = 0;
_Bool transmit = 0;

struct axradio_address rf_destination;

nbfi_rf_state_s rf_state = STATE_OFF;

nbfi_phy_channel_t nbfi_phy_channel;

uint8_t PSK_BAND;
uint8_t DBPSK_BAND;

NBFi_ax5043_pins_s  nbfi_ax5043_pins;

void    NBFi_TX_Finished();
void    NBFi_ParseReceivedPacket(struct axradio_status *st);
void    ax5043_set_constants(void);



void axradio_statuschange(struct axradio_status  *st)
{
    switch (st->status)
    {
    case AXRADIO_STAT_TRANSMITSTART:
        axradio_set_channel(0);
        if( st->error == AXRADIO_ERR_TIMEOUT )  rf_busy = 0;
        break;

    case AXRADIO_STAT_TRANSMITEND:
        transmit = 0;
        NBFi_TX_Finished();
        break;

    case AXRADIO_STAT_RECEIVE:
        if(st->error == AXRADIO_ERR_NOERROR)
        {
            NBFi_ParseReceivedPacket(st);
        }
        break;

    default:
        break;
    }
}


void RF_SetFreq(uint32_t freq)
{
    axradio_phy_chanfreq[0] = axradio_conv_freq_fromhz(freq);
    // Dirty fix for insufficient arithmetics precision
    if(freq > 800000000)    axradio_phy_chanfreq[0]+=34; //868MHz
    else                    axradio_phy_chanfreq[0]+=18; //446MHz
}


const uint16_t AX5043_power[26] = {0x00aa, 0x00bf, 0x00d1, 0x00ec, 0x010f, 0x0132, 0x0156, 0x017f, 0x01af, 0x1e0, 0x207, 0x244, 0x290, 0x2eb, 0x35e, 0x3d6, 0x406, 0x4a9, 0x57c, 0x600, 0x700, 0x800, 0x9d4, 0xc00, 0xf00, 0xfff};


struct axradio_address  fastdladdress = {
	{ 0x6f, 0x6f, 0x6f, 0x6f}
};

extern void (* __nbfi_before_tx)(NBFi_ax5043_pins_s *);
extern void (* __nbfi_before_rx)(NBFi_ax5043_pins_s *);
extern void (* __nbfi_before_off)(NBFi_ax5043_pins_s *);

void RF_SetModeAndPower(int8_t dBm, rf_direction_t mode, rf_antenna_t ant)
{
    switch(mode)
    {
    case TX:    
      nbfi_ax5043_pins.txpwr = AX5043_power[dBm + 10];
      nbfi_ax5043_pins.cfga = PA_DIFFERENTIAL;            
      if(__nbfi_before_tx) __nbfi_before_tx(&nbfi_ax5043_pins);
      break;
    case RX: 
      nbfi_ax5043_pins.cfga = PA_DIFFERENTIAL;
      if(__nbfi_before_rx) __nbfi_before_rx(&nbfi_ax5043_pins);
      break;
    case IDLE:
      if(__nbfi_before_off)   __nbfi_before_off(&nbfi_ax5043_pins);
      break;
    }

}

nbfi_status_t RF_Init(  nbfi_phy_channel_t  phy_channel,
                        rf_antenna_t        antenna,
                        int8_t              power,
                        uint32_t            freq)
{
    uint8_t er;

    if(rf_busy) return ERR_RF_BUSY;

    rf_busy = 1;

    if(phy_channel != OSC_CAL) nbfi_phy_channel = phy_channel;

    if(freq > 600000000) PSK_BAND = PSK_864;
    else PSK_BAND = PSK_446_DBPSK_868;

    if((freq < 500000000)&&(freq > 450000000)) DBPSK_BAND = PSK_446_DBPSK_458;
    else DBPSK_BAND = PSK_446_DBPSK_868; 
    
    ax5043_hard_reset();
    
    
    ax5043_tcxo_set_reset(1);


    switch(phy_channel)
    {
    case UL_PSK_200:
    case UL_PSK_FASTDL:
    case UL_PSK_500:
    case UL_PSK_5000:
    case UL_DBPSK_50_PROT_D:
    case UL_DBPSK_400_PROT_D:
    case UL_DBPSK_3200_PROT_D:
    case UL_DBPSK_25600_PROT_D:
        ax5043_set_constants();
        
        RF_SetModeAndPower(power, TX, antenna);
        
        RF_SetFreq(freq);
        
        er = axradio_init();    // Init radio registers
        if (er != AXRADIO_ERR_NOERROR)
        {
            rf_busy = 0; return ERR;
        }
        er = axradio_set_mode(AXRADIO_MODE_ASYNC_TRANSMIT);
        if (er != AXRADIO_ERR_NOERROR)
        {
            rf_busy = 0;return ERR;
        }
        
        rf_busy = 0;
        rf_state = STATE_TX;
        return OK;


    case DL_PSK_200:
    case DL_PSK_FASTDL:
    case DL_PSK_500:
    case DL_PSK_5000:
        ax5043_set_constants();
        
        RF_SetModeAndPower(power, RX, antenna);

        RF_SetLocalAddress((uint8_t *)&fastdladdress);

        RF_SetFreq(freq);

        er = axradio_init();    // Init radio registers
        if (er != AXRADIO_ERR_NOERROR)
        {
             rf_busy = 0; return ERR;
        }
        er = axradio_set_mode(AXRADIO_MODE_ASYNC_RECEIVE);
        rf_busy = 0;
        if (er != AXRADIO_ERR_NOERROR)
        {
            return ERR;
        }
        rf_state = STATE_RX;
        return OK;
    case OSC_CAL:
        axradio_set_mode(AXRADIO_MODE_ASYNC_RECEIVE);
        axradio_set_mode(AXRADIO_MODE_OFF);
        delay_ms(2);
        rf_state = STATE_OFF;

    }
    ax5043_tcxo_set_reset(0);
    rf_busy = 0;
    return ERR;
}

extern void (*__ax5043_on_off_pwr)(uint8_t);
nbfi_status_t RF_Deinit()
{
    uint8_t er;
    if(rf_busy) return ERR_RF_BUSY;
    RF_SetModeAndPower(0, IDLE, PCB);
    rf_busy = 1;
    ax5043_spi_write(AX5043_PWRMODE, 0x80);
    ax5043_spi_write(AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);
    ax5043_set_registers(); 
    er = axradio_set_mode(AXRADIO_MODE_OFF);
    rf_busy = 0;
    transmit = 0;
    RF_SetModeAndPower(0, RX, PCB);
    delay_ms(1);
    ax5043_tcxo_set_reset(0);
    if(__ax5043_on_off_pwr) __ax5043_on_off_pwr(0);
    rf_state = STATE_OFF;
    if (er != AXRADIO_ERR_NOERROR) return ERR;
    return OK;
}

void RF_SetDstAddress(uint8_t * addr)
{
    for(uint8_t i = 0; i !=3; i++) rf_destination.addr[i] = addr[i];
}


void RF_SetLocalAddress(uint8_t * addr)
{
    struct axradio_address_mask     localaddress = {{0,0,0,0},{0xff, 0, 0, 0}};

    localaddress.addr[0] = addr[0];

    axradio_set_local_address(&localaddress);
}


nbfi_status_t RF_Transmit(uint8_t* pkt, uint8_t len,  rf_padding_t padding, rf_blocking_t blocking)
{
    if(rf_busy) return ERR_RF_BUSY;

    rf_busy = 1;

    axradio_transmit(&rf_destination, pkt, len, padding);
    
    rf_busy = 0;
    
    transmit = 1;
    
    if(blocking == BLOCKING)
    {

        while(1) // Wait for TX complete
        {
            if(!transmit) break;
            wtimer_runcallbacks();
        }

    }

    return OK;
}






