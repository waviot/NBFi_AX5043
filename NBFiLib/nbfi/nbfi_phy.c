#include "nbfi.h"
#include "nbfi_config.h"
#include "nbfi_misc.h"
#include "nbfi_phy.h"
#include "rf.h"
#include "xtea.h"
#include "zcode.h"
#include "zcode_e.h"
#include <string.h>
#include <stdlib.h> 

#define memset_xdata memset
#define memcpy_xdata memcpy
#define memcpy_xdatageneric memcpy
#define memcpy_genericxdata memcpy

uint32_t last_pkt_crc = 0;

const uint8_t protD_preambula[] = {0x97, 0x15, 0x7A, 0x6F};


uint32_t tx_freq, rx_freq;

extern nbfi_state_t nbfi_state;
extern nbfi_transport_packet_t* nbfi_active_pkt;
extern _Bool wait_RxEnd;

/*
nbfi_status_t NBFi_TX_ProtocolE(nbfi_transport_packet_t* pkt)
{
    uint8_t ul_buf[64];
    uint8_t len = 0;
    static _Bool parity = 0;
    uint16_t lastcrc16;

    memset_xdata(ul_buf,0,sizeof(ul_buf));

    if(nbfi.mode == TRANSPARENT) pkt->phy_data_length--;

    for(int i=0; i<sizeof(protD_preambula); i++)
    {
        ul_buf[len++] = protD_preambula[i];
    }

    ul_buf[len++] = nbfi.full_ID[0];
    ul_buf[len++] = nbfi.full_ID[1];
    ul_buf[len++] = nbfi.full_ID[2];
    ul_buf[len++] = nbfi.full_ID[3];

    if(nbfi.tx_phy_channel == DL_DBPSK_50_PROT_E) nbfi.tx_phy_channel = UL_DBPSK_50_PROT_E;
    else if(nbfi.tx_phy_channel == DL_DBPSK_400_PROT_E) nbfi.tx_phy_channel = UL_DBPSK_400_PROT_E;
    else if(nbfi.tx_phy_channel == DL_DBPSK_3200_PROT_E) nbfi.tx_phy_channel = UL_DBPSK_3200_PROT_E;
    else if(nbfi.tx_phy_channel == DL_DBPSK_25600_PROT_E) nbfi.tx_phy_channel = UL_DBPSK_25600_PROT_E;

    ul_buf[len++] = pkt->phy_data.header;

    memcpy_xdatageneric(&ul_buf[len], pkt->phy_data.payload, pkt->phy_data_length);

    lastcrc16 =  CRC16(&ul_buf[len], 8, 0xFFFF);

    if(XTEA_Enabled() && XTEA_Available() && !(nbfi.additional_flags&NBFI_FLG_NO_XTEA))
    {
        XTEA_Encode(&ul_buf[len]);
    }
    len += 8;

    if(nbfi.mode == TRANSPARENT)
    {
        ul_buf[len++] = pkt->phy_data.payload[8];
        ul_buf[len++] = pkt->phy_data.payload[9];
    }
    else
    {
        ul_buf[len++] = lastcrc16&0xff;
        ul_buf[len++] = (lastcrc16>>8)&0xff;
    }

    last_pkt_crc = CRC32(ul_buf + 4, 15); 

    ul_buf[len++] = (uint8_t)(last_pkt_crc >> 16);
    ul_buf[len++] = (uint8_t)(last_pkt_crc >> 8);
    ul_buf[len++] = (uint8_t)(last_pkt_crc);

    if(nbfi.tx_freq)
    {
        tx_freq = nbfi.tx_freq ;
        parity = (nbfi.tx_freq > (nbfi.ul_freq_base + 25000));
    }
    else
    {
        if(nbfi.tx_phy_channel < UL_DBPSK_3200_PROT_D)
        {
                tx_freq = nbfi.ul_freq_base + (((*((const uint32_t *)FULL_ID)+lastcrc16)%226)*100);
                if(parity) tx_freq = tx_freq + 27500;
        }
        else
        {
            tx_freq = nbfi.ul_freq_base + 1600 + (((*((const uint32_t *)FULL_ID)+lastcrc16)%210)*100);
            if(parity) tx_freq = tx_freq + 27500 - 1600;
        }
    }

    ZCODE_E_Append(&ul_buf[4], &ul_buf[len], 1);

    if(!nbfi.tx_freq) parity = !parity;

    if((nbfi.mode == NRX) && parity)
    {
        RF_Init(nbfi.tx_phy_channel, (rf_antenna_t)nbfi.tx_antenna, nbfi.tx_pwr, tx_freq);
        RF_Transmit(ul_buf, len + ZCODE_LEN, PADDING_4TO1, BLOCKING);
        nbfi_state.UL_total++;
        return NBFi_TX_ProtocolE(pkt);
    }

    RF_Init(nbfi.tx_phy_channel, (rf_antenna_t)nbfi.tx_antenna, nbfi.tx_pwr, tx_freq);

    RF_Transmit(ul_buf, len + ZCODE_LEN, PADDING_4TO1, NONBLOCKING);

    nbfi_state.UL_total++;

    return OK;

}
*/

nbfi_status_t NBFi_TX_ProtocolD(nbfi_transport_packet_t* pkt)
{
    
    uint8_t ul_buffer[64+8];

    uint8_t *ul_buf = &ul_buffer[8];

    uint8_t len = 0;
    static _Bool parity = 0;
    uint8_t lastcrc8;
    _Bool downlink;

    memset_xdata(ul_buffer,0, sizeof(ul_buffer));

    if(nbfi.mode == TRANSPARENT) pkt->phy_data_length--;

    for(int i=0; i<sizeof(protD_preambula); i++)
    {
        ul_buf[len++] = protD_preambula[i];
    }

    switch(nbfi.tx_phy_channel)
    {
        case DL_DBPSK_50_PROT_D:
        case DL_DBPSK_400_PROT_D:
        case DL_DBPSK_3200_PROT_D:
        case DL_DBPSK_25600_PROT_D:
            ul_buf[len++] = nbfi.dl_ID[0];
            ul_buf[len++] = nbfi.dl_ID[1];
            ul_buf[len++] = nbfi.dl_ID[2];
            downlink = 1;
            break;
        default:
            ul_buf[len++] = nbfi.temp_ID[0];
            ul_buf[len++] = nbfi.temp_ID[1];
            ul_buf[len++] = nbfi.temp_ID[2];
            downlink = 0;
            break;

    }

    if(nbfi.tx_phy_channel == DL_DBPSK_50_PROT_D) nbfi.tx_phy_channel = UL_DBPSK_50_PROT_D;
    else if(nbfi.tx_phy_channel == DL_DBPSK_400_PROT_D) nbfi.tx_phy_channel = UL_DBPSK_400_PROT_D;
    else if(nbfi.tx_phy_channel == DL_DBPSK_3200_PROT_D) nbfi.tx_phy_channel = UL_DBPSK_3200_PROT_D;
    else if(nbfi.tx_phy_channel == DL_DBPSK_25600_PROT_D) nbfi.tx_phy_channel = UL_DBPSK_25600_PROT_D;

    ul_buf[len++] = pkt->phy_data.header;

    memcpy_xdatageneric(&ul_buf[len], pkt->phy_data.payload, pkt->phy_data_length);

    lastcrc8 =  CRC8(&ul_buf[len], 8);

    if(XTEA_Enabled() && XTEA_Available() && !(nbfi.additional_flags&NBFI_FLG_NO_XTEA))
    {
        XTEA_Encode(&ul_buf[len]);
    }
    len += 8;

    if(nbfi.mode == TRANSPARENT)
    {
        ul_buf[len++] = pkt->phy_data.payload[8];
    }
    else  ul_buf[len++] = lastcrc8;

    last_pkt_crc = CRC32(ul_buf + 4, 13); 

    ul_buf[len++] = (uint8_t)(last_pkt_crc >> 16);
    ul_buf[len++] = (uint8_t)(last_pkt_crc >> 8);
    ul_buf[len++] = (uint8_t)(last_pkt_crc);

    if(nbfi.tx_freq)
    {
        tx_freq = nbfi.tx_freq ;
        parity = (nbfi.tx_freq > (nbfi.ul_freq_base + 25000));
    }
    else
    {
        switch(nbfi.tx_phy_channel)
        {
            case UL_DBPSK_3200_PROT_D:
              tx_freq = nbfi.ul_freq_base + 1600 + (((*((const uint32_t*)FULL_ID)+lastcrc8)%210)*100);
              if(parity) tx_freq = tx_freq + 27500 - 1600;
            break;
            case UL_DBPSK_25600_PROT_D:
              tx_freq = nbfi.ul_freq_base + 12800;
              if(parity) tx_freq = tx_freq + 25600;
            default:
              tx_freq = nbfi.ul_freq_base + (((*((const uint32_t*)FULL_ID)+lastcrc8)%226)*100);
              if(parity) tx_freq = tx_freq + 27500;
            break;
        }
    }

    if((nbfi.tx_phy_channel < UL_DBPSK_3200_PROT_D) && !downlink)
    {
                ZCODE_Append(&ul_buf[4], &ul_buf[len], parity);
    }
    else
    {
                ZCODE_Append(&ul_buf[4], &ul_buf[len], 1);
    }

    if(!nbfi.tx_freq) parity = !parity;

    if((nbfi.mode == NRX) && parity && (!nbfi.tx_freq)) // For NRX send in ALOHA mode
    {
      
      RF_Init(nbfi.tx_phy_channel, (rf_antenna_t)nbfi.tx_antenna, nbfi.tx_pwr, tx_freq);
      
      RF_Transmit(ul_buf, len + ZCODE_LEN, PADDING_4TO1, BLOCKING);
      
      nbfi_state.UL_total++;
      
      return NBFi_TX_ProtocolD(pkt);
    }
    
    RF_Init(nbfi.tx_phy_channel, (rf_antenna_t)nbfi.tx_antenna, nbfi.tx_pwr, tx_freq);

    
    switch (nbfi.tx_phy_channel)
    {
        case UL_DBPSK_3200_PROT_D:
            RF_Transmit(ul_buffer + 7, len + ZCODE_LEN + 1, PADDING_4TO1, NONBLOCKING);
            break;
        case UL_DBPSK_25600_PROT_D:
            RF_Transmit(ul_buffer + 3, len + ZCODE_LEN + 5, PADDING_4TO1, NONBLOCKING);
            break;
        default:
            RF_Transmit(ul_buffer + 8, len + ZCODE_LEN, PADDING_4TO1, NONBLOCKING);
            break;
    }
    

    nbfi_state.UL_total++;

    return OK;

}

_Bool NBFi_Match_ID(uint8_t * addr)
{
    uint8_t i;
    for( i = 0; i !=3; i++) if(nbfi.temp_ID[i] != addr[i]) break;
    if(i == 3)  return 1;

    for(i = 0; i !=3; i++) if(nbfi.broadcast_ID[i] != addr[i]) break;
    if(i == 3)  return 1;

    return 0;
}

nbfi_status_t NBFi_TX(nbfi_transport_packet_t* pkt)
{
    nbfi_status_t result;

    if((pkt->phy_data_length==0)&&(pkt->phy_data_length>240)) return ERR; // len check
    switch(nbfi.tx_phy_channel)
    {
    case UL_DBPSK_50_PROT_D:
    case UL_DBPSK_400_PROT_D:
    case UL_DBPSK_3200_PROT_D:
    case UL_DBPSK_25600_PROT_D:
    case DL_DBPSK_50_PROT_D:
    case DL_DBPSK_400_PROT_D:
    case DL_DBPSK_3200_PROT_D:
    case DL_DBPSK_25600_PROT_D:
        return NBFi_TX_ProtocolD(pkt);
    case UL_PSK_FASTDL:
    case UL_PSK_200:
    case UL_PSK_500:
    case UL_PSK_5000:

        if(nbfi.tx_freq == 0) tx_freq = nbfi.dl_freq_base;
        else tx_freq = nbfi.tx_freq;

        if(nbfi.tx_phy_channel == UL_PSK_FASTDL)
        {
            tx_freq += 1000000;
        }

        RF_SetDstAddress((uint8_t *)&fastdladdress);

        if((result = RF_Init(nbfi.tx_phy_channel, (rf_antenna_t)nbfi.tx_antenna, nbfi.tx_pwr, tx_freq)) != OK) return result;

        uint8_t* buf = (uint8_t*) malloc(pkt->phy_data_length + 1 + 3);
        if(!buf) return ERR_BUFFER_FULL;
        if(nbfi.dl_ID[0] == 0)
        {
            for(uint8_t i = 0; i != 2; i++) buf[i] = nbfi.dl_ID[i + 1];
            memcpy_xdata(buf + 2, (const void *)&pkt->phy_data.header, pkt->phy_data_length + 1);
            result = RF_Transmit(buf, pkt->phy_data_length + 1 + 2, NO_PADDING, NONBLOCKING);
        }
        else
        {
            for(uint8_t i = 0; i != 3; i++) buf[i] = nbfi.dl_ID[i];
            memcpy_xdata(buf + 3, (const void *)&pkt->phy_data.header, pkt->phy_data_length + 1);
            result = RF_Transmit(buf, pkt->phy_data_length + 1 + 3, NO_PADDING, NONBLOCKING);
        }

        free(buf);
        if(result != OK) return result;
        nbfi_state.UL_total++;
        break;

    }

    return OK;
}


nbfi_status_t NBFi_RX_Controller()
{
    switch(nbfi.mode)
    {
    case  DRX:
    case  NRX:
        if(wait_RxEnd ) if(rf_state != STATE_RX)return NBFi_RX();
        else break;
        switch(nbfi_active_pkt->state)
        {
        case PACKET_WAIT_ACK:
        case PACKET_QUEUED_AGAIN:
        case PACKET_WAIT_FOR_EXTRA_PACKETS:
            if(rf_state != STATE_RX) return NBFi_RX();
            break;
        default:
            if(rf_state != STATE_OFF)  return RF_Deinit();
        }
        break;
    case CRX:
    case TRANSPARENT:
        if(rf_state != STATE_RX) return NBFi_RX();
        break;
    case OFF:
        if(rf_state != STATE_OFF)  return RF_Deinit();
        break;
    }
    return OK;
}

nbfi_status_t NBFi_RX()
{
    nbfi_status_t result;
    switch(nbfi.rx_phy_channel)
    {
        case DL_PSK_200:
        case DL_PSK_500:
        case DL_PSK_5000:
        case DL_PSK_FASTDL:
             if(nbfi.rx_freq == 0) rx_freq = nbfi.dl_freq_base + ((*((const uint32_t*)FULL_ID)%276)*363);
             else rx_freq = nbfi.rx_freq;
             if(nbfi.rx_phy_channel == DL_PSK_FASTDL) rx_freq += 1000000;
             break;
    }
    result = RF_Init(nbfi.rx_phy_channel, (rf_antenna_t)nbfi.rx_antenna, 0, rx_freq);
    return result;
}


void NBFi_XTEA_OFB(uint8_t* buf, uint8_t len, uint8_t iter)
{
 uint8_t vector[8];
 for(uint8_t i = 0; i != 3; i++)
 {
    vector[i] = 0;
    vector[i+5] = nbfi.temp_ID[i];
 }
 vector[3] = 0;
 vector[4] = iter;

 uint8_t n = 0;// number of cyphered bytes

 while(n < len)
 {

  if((n % 8) == 0) XTEA_Encode(vector); // next block

  buf[n] = vector[n%8] ^ buf[n];
  n++;
 }
}
