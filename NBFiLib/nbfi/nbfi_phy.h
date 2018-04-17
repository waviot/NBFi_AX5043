#ifndef NBFI_PHY_H
#define NBFI_PHY_H
#include "nbfi_misc.h"

_Bool            NBFi_Match_ID(uint8_t * addr);
nbfi_status_t   NBFi_TX_ProtocolE(nbfi_transport_packet_t* pkt);
nbfi_status_t   NBFi_TX_ProtocolD(nbfi_transport_packet_t* pkt);
nbfi_status_t   NBFi_TX_ProtocolC(nbfi_transport_packet_t* pkt);
nbfi_status_t   NBFi_TX_Correction();
nbfi_status_t   NBFi_RX_Controller();
nbfi_status_t   NBFi_RX();
nbfi_status_t   NBFi_TX(nbfi_transport_packet_t* pkt);
void            NBFi_XTEA_OFB(uint8_t* buf, uint8_t len, uint8_t iter);

#endif