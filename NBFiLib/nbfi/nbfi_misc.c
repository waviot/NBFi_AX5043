#include "nbfi.h"
#include "nbfi_config.h"
#include "nbfi_misc.h"
#include "rf.h"
#include <stdlib.h>
#include <string.h>

nbfi_transport_packet_t * nbfi_TX_pktBuf[NBFI_TX_PKTBUF_SIZE];
nbfi_transport_packet_t* nbfi_RX_pktBuf[NBFI_RX_PKTBUF_SIZE];

uint8_t     nbfi_TXbuf_head = 0;

extern nbfi_state_t nbfi_state;
extern nbfi_transport_packet_t idle_pkt;
extern nbfi_transport_packet_t* nbfi_active_pkt;


nbfi_transport_packet_t* NBFi_AllocateTxPkt(uint8_t payload_length)
{
    uint8_t ptr = nbfi_TXbuf_head%NBFI_TX_PKTBUF_SIZE;

    if(nbfi_TX_pktBuf[ptr])
    {
        switch(nbfi_TX_pktBuf[ptr]->state)
        {
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
        case PACKET_WAIT_ACK:
        case PACKET_NEED_TO_SEND_RIGHT_NOW:
        case PACKET_SENT_NOACKED:
            return 0;   // tx buffer is full
        }
        free(nbfi_TX_pktBuf[ptr]);
        nbfi_TX_pktBuf[ptr] = 0;
    }

    nbfi_TX_pktBuf[ptr] = (nbfi_transport_packet_t *) malloc(sizeof(nbfi_transport_packet_t) + payload_length);

    if(!nbfi_TX_pktBuf[ptr])
    {
        return 0;
    }

    nbfi_TX_pktBuf[ptr]->state = PACKET_ALLOCATED;

    nbfi_TX_pktBuf[ptr]->phy_data_length = payload_length;

    nbfi_TX_pktBuf[ptr]->handshake = HANDSHAKE_NONE;

    nbfi_TX_pktBuf[ptr]->retry_num = 0;

    nbfi_TX_pktBuf[ptr]->mack_num = 0;

    nbfi_TX_pktBuf[ptr]->phy_data.header = 0;

    nbfi_TXbuf_head++;

    return nbfi_TX_pktBuf[ptr];

}



nbfi_transport_packet_t* NBFi_AllocateRxPkt(uint8_t header, uint8_t payload_length)
{
    uint8_t ptr = header&0x1f;

    switch(nbfi_active_pkt->state)
    {
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
        case PACKET_WAIT_ACK:
            nbfi_active_pkt = &idle_pkt;
            break;

    }

    if(nbfi_RX_pktBuf[ptr])
    {
        free(nbfi_RX_pktBuf[ptr]);
    }

    nbfi_RX_pktBuf[ptr] = (nbfi_transport_packet_t *) malloc(sizeof(nbfi_transport_packet_t) + payload_length);

    if(!nbfi_RX_pktBuf[ptr])
    {
        return 0;
    }

    nbfi_state.DL_iter = ptr;

    nbfi_RX_pktBuf[ptr]->state = PACKET_ALLOCATED;

    nbfi_RX_pktBuf[ptr]->phy_data_length = payload_length;

    nbfi_RX_pktBuf[ptr]->phy_data.header = header;

    return nbfi_RX_pktBuf[ptr];

}


nbfi_transport_packet_t* NBFi_GetQueuedTXPkt()
{

    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        switch(nbfi_TX_pktBuf[ptr]->state )
        {
        case PACKET_QUEUED_AGAIN:
        case PACKET_NEED_TO_SEND_RIGHT_NOW:
            return nbfi_TX_pktBuf[ptr];
        }
    }

    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        switch(nbfi_TX_pktBuf[ptr]->state )
        {
        case PACKET_WAIT_ACK:
            if(nbfi_TX_pktBuf[ptr] == nbfi_active_pkt) continue;
             nbfi_TX_pktBuf[ptr]->state = PACKET_QUEUED_AGAIN;
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
            return nbfi_TX_pktBuf[ptr];
        }
    }
    return 0;
}

void NBFi_TxPacket_Free(nbfi_transport_packet_t* pkt)
{
    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] != pkt) continue;
        free(pkt);
        nbfi_TX_pktBuf[ptr] = 0;
    }

}

void NBFi_RxPacket_Free(nbfi_transport_packet_t* pkt)
{
    for(uint8_t i = 0; i != NBFI_RX_PKTBUF_SIZE; i++)
    {
        if(nbfi_RX_pktBuf[i] != pkt) continue;
        free(pkt);
        nbfi_RX_pktBuf[i] = 0;
    }

}

uint8_t NBFi_Packets_To_Send()
{

    uint8_t packets_free = 0;

    for(uint16_t i = nbfi_TXbuf_head; i != (nbfi_TXbuf_head + NBFI_TX_PKTBUF_SIZE); i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0)
        {
            packets_free++;
            continue;
        }
        switch(nbfi_TX_pktBuf[ptr]->state )
        {
        case PACKET_WAIT_ACK:
            if(nbfi_TX_pktBuf[ptr] == nbfi_active_pkt) break;
             nbfi_TX_pktBuf[ptr]->state = PACKET_QUEUED_AGAIN;
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
        case PACKET_NEED_TO_SEND_RIGHT_NOW:
        case PACKET_SENT_NOACKED:
            break;
        default:
            packets_free++;
            continue;
        }
        break;
    }

    if((transmit == 1) && (packets_free == NBFI_TX_PKTBUF_SIZE))
    {
        packets_free--;
    }

    return NBFI_TX_PKTBUF_SIZE - packets_free;
}

void NBFi_Mark_Lost_All_Unacked()
{
    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        if(nbfi_TX_pktBuf[ptr]->state == PACKET_SENT_NOACKED) nbfi_TX_pktBuf[ptr]->state = PACKET_LOST;
    }
}

uint8_t NBFi_Calc_Packets_With_State(uint8_t state)
{
    uint8_t num = 0;
    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        if(nbfi_TX_pktBuf[ptr]->state == state) num++;
    }
    return num;
}

uint8_t NBFi_Calc_Queued_Sys_Packets_With_Type(uint8_t type)
{
    uint8_t num = 0;
    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        if(!nbfi_TX_pktBuf[ptr]->phy_data.SYS) continue;
        if(nbfi_TX_pktBuf[ptr]->phy_data.payload[0] != type) continue;

        switch(nbfi_TX_pktBuf[ptr]->state )
        {
        case PACKET_WAIT_ACK:
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
        case PACKET_NEED_TO_SEND_RIGHT_NOW:
        case PACKET_SENT_NOACKED:
            num++;
            break;
        }
    }
    return num;
}


nbfi_transport_packet_t* NBFi_GetSentTXPkt_By_Iter(uint8_t iter)
{

    for(uint8_t i = (nbfi_TXbuf_head) - 1; i != (uint8_t)(nbfi_TXbuf_head  - NBFI_TX_PKTBUF_SIZE - 1); i--)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        if(nbfi_TX_pktBuf[ptr]->phy_data.SYS && (nbfi_TX_pktBuf[ptr]->phy_data.payload[0] != 0x05)&&(nbfi_TX_pktBuf[ptr]->phy_data.payload[0] != 0x02)&& !(nbfi_TX_pktBuf[ptr]->phy_data.payload[0] & 0x80))
        {
            continue;
        }
        switch(nbfi_TX_pktBuf[ptr]->state)
        {
        case PACKET_SENT:
        case PACKET_SENT_NOACKED:
        case PACKET_WAIT_ACK:
        case PACKET_LOST:
        case PACKET_DELIVERED:
            if(nbfi_TX_pktBuf[ptr]->phy_data.ITER == iter)
            {
                return nbfi_TX_pktBuf[ptr];
            }
            break;
        }

    }
    return 0;
}


uint32_t NBFi_Get_RX_ACK_Mask()
{
    uint32_t mask = 0;
    uint32_t one = 1;
    for(uint8_t i = ((nbfi_state.DL_iter - 1)&0x1f); (i&0x1f) != (nbfi_state.DL_iter&0x1f); i-- , one <<= 1 )
    {
        if(!nbfi_RX_pktBuf[i&0x1f]) continue;
        switch(nbfi_RX_pktBuf[i&0x1f]->state)
        {
            case PACKET_RECEIVED:
            case PACKET_PROCESSED:
                mask |= one;
                break;
            default:
                break;
        }
    }
    return mask;
}

_Bool NBFi_Check_RX_Packet_Duplicate(nbfi_pfy_packet_t * pkt, uint8_t len)
{
    nbfi_pfy_packet_t *rec_pkt = &nbfi_RX_pktBuf[nbfi_state.DL_iter&0x1f]->phy_data;

    for(uint8_t i = 0; i != len; i++)
    {
        if(((uint8_t*)rec_pkt)[i] != ((uint8_t*)pkt)[i]) return 0;
    }
    return 1;
}


nbfi_transport_packet_t* NBFi_Get_QueuedRXPkt(uint8_t *groupe, uint16_t *total_length)
{
    nbfi_transport_packet_t* pkt;

    uint32_t i;
    for(i = nbfi_state.DL_iter + 1; i <= (nbfi_state.DL_iter + NBFI_RX_PKTBUF_SIZE*2); i++ )
    {
        *groupe = 0;
        *total_length = 0;
        uint8_t total_groupe_len = 0;

        while((pkt = nbfi_RX_pktBuf[(i + *groupe)&0x1f]) && pkt->state == PACKET_RECEIVED)
        {
            if((*groupe) == 0)
            {
                *groupe = 1;

                if((pkt->phy_data.MULTI)&&(pkt->phy_data.SYS)&&((pkt->phy_data.payload[0] == 0x05)||(pkt->phy_data.payload[0] == 0x02))) //the start packet of the groupe
                {
                    total_groupe_len = pkt->phy_data.payload[1];
                    *total_length = pkt->phy_data_length - 2;
                    continue;
                }
                else
                {
                    if((pkt->phy_data.MULTI == 1)&&!(pkt->phy_data.SYS)) break;
                    //single packet
                    *total_length = pkt->phy_data_length;
                    pkt->state = PACKET_PROCESSING;
                    break;
                }
            }
            (*total_length) += pkt->phy_data_length;
            (*groupe)++;

            if((pkt->phy_data.MULTI) && ((*total_length) < total_groupe_len) && ((*groupe) < NBFI_RX_PKTBUF_SIZE - 1))
            {
                    continue;
            }
            else
            {       if(pkt->phy_data.MULTI && ((*total_length) >= total_groupe_len))
                    {
                        (*total_length) = total_groupe_len;
                        pkt->state = PACKET_PROCESSING;
                        break;
                    }

            }
            break;

        }

        if((*groupe) && (nbfi_RX_pktBuf[(i + (*groupe) - 1)&0x1f]->state == PACKET_PROCESSING))
        {
            return nbfi_RX_pktBuf[i&0x1f];
        }

    }
    return 0;
}

void NBFi_Clear_RX_Buffer()
{
    for(uint8_t i = 0; i != NBFI_RX_PKTBUF_SIZE; i++ )
    {
        nbfi_RX_pktBuf[i]->state = PACKET_CLEARED;
    }
}

extern nbfi_transport_packet_t idle_pkt;
extern nbfi_transport_packet_t* nbfi_active_pkt;
void NBFi_Clear_TX_Buffer()
{
    for(uint8_t i = 0; i != NBFI_TX_PKTBUF_SIZE; i++ )
    {
        if(nbfi_TX_pktBuf[i])
        {
            free(nbfi_TX_pktBuf[i]);
            nbfi_TX_pktBuf[i] = 0;
        }
    }
    nbfi_active_pkt = &idle_pkt;
}

extern uint8_t nbfi_last_snr;
extern int16_t noise;
extern uint8_t you_should_dl_power_step_down;
extern uint8_t you_should_dl_power_step_up;
void NBFi_Send_Clear_Cmd(uint8_t iter)
{
    nbfi_transport_packet_t* pkt =  NBFi_AllocateTxPkt(8);
    if(!pkt) return;
    pkt->phy_data.payload[0] = 0x08; //clear RX buffer
    
    pkt->phy_data.payload[5] = nbfi_last_snr;
    pkt->phy_data.payload[6] = (uint8_t)(noise + 150);
    pkt->phy_data.payload[7] = you_should_dl_power_step_down + you_should_dl_power_step_up + (nbfi.tx_pwr & 0x3f);
    pkt->phy_data.ITER = iter;
    pkt->phy_data.header |= SYS_FLAG;
    pkt->handshake = HANDSHAKE_NONE;
    pkt->state = PACKET_NEED_TO_SEND_RIGHT_NOW;
}


void NBFi_Resend_Pkt(nbfi_transport_packet_t* act_pkt, uint32_t mask)
{
    uint8_t iter = act_pkt->phy_data.ITER;
    uint32_t selection;
    mask = (~mask) << 1;
    selection = 0;
    for(uint8_t i = (act_pkt->mack_num&0x3f) - 1; i != 0; i-- )
    {
      selection |= (((uint32_t)1) << i);
    }
    mask &= selection;
    uint32_t one = 1;

    nbfi_transport_packet_t* pkt = 0, *last_resending_pkt = 0;
    
    for(uint8_t i = (act_pkt->mack_num&0x3f); i > 0; i--)
    {
        pkt = NBFi_GetSentTXPkt_By_Iter(iter&0x1f);

        if(!pkt)
        {
          break;
        }
        if(one&mask)
        {
          mask &= ~one;
          pkt->state = PACKET_QUEUED_AGAIN;
          if(last_resending_pkt == 0) last_resending_pkt = pkt;
          nbfi_state.fault_total++;
        }
        else
        {
          pkt->state = PACKET_DELIVERED;
          nbfi_state.success_total++;
        }
        iter--;
        one <<= 1;
    }
    if(last_resending_pkt)
    {
        last_resending_pkt->phy_data.ACK = 1;
        last_resending_pkt->mack_num = act_pkt->mack_num - (act_pkt->phy_data.ITER - last_resending_pkt->phy_data.ITER);
        if(act_pkt->phy_data.ITER < last_resending_pkt->phy_data.ITER) last_resending_pkt->mack_num += 32;
        last_resending_pkt->mack_num |= 0x80;
    }
    else if((act_pkt->mack_num > 1) && (mask == 0))  //all packets delivered, send message to clear receiver's input buffer
    {
         NBFi_Send_Clear_Cmd(nbfi_active_pkt->phy_data.ITER);
    }

}


#define POLY 0xa001

uint16_t CRC16(uint8_t *buf, uint16_t len, uint16_t crc)
{
    while (len--)
    {
        crc ^= *buf++;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
    }
    return crc;
}

static unsigned char CRC8byte(unsigned char data)
{
   uint8_t crc = 0;
   if(data & 1)     crc ^= 0x5e;
   if(data & 2)     crc ^= 0xbc;
   if(data & 4)     crc ^= 0x61;
   if(data & 8)     crc ^= 0xc2;
   if(data & 0x10)  crc ^= 0x9d;
   if(data & 0x20)  crc ^= 0x23;
   if(data & 0x40)  crc ^= 0x46;
   if(data & 0x80)  crc ^= 0x8c;
   return crc;
}

uint8_t CRC8(uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        crc = CRC8byte(data[i] ^ crc);
    }
    return crc;
}


//#define CRC_TABLE

#ifndef CRC_TABLE

#define WIDTH (8*4)
#define TOPBIT (1 << (WIDTH-1))
#define POLYNOMIAL (0x104C11DB7)

uint32_t crc_table(uint8_t n)
{
    uint32_t c;
    int k;
    c=((uint32_t)n) << (WIDTH - 8);
    for(k=8;k>0;k--)
    {
        if(c & TOPBIT)
        {
            c = (c<<1) ^ POLYNOMIAL;
        }
        else
        {
            c=c<<1;
        }
    }
    return c;
}
#endif

// polynomial: 0x104C11DB7
uint32_t digital_update_crc32( uint32_t crc, const uint8_t *data, uint8_t len )
{
#ifdef CRC_TABLE
    static const uint32_t crc_table[256] = {
        0x00000000U,0x04C11DB7U,0x09823B6EU,0x0D4326D9U,
        0x130476DCU,0x17C56B6BU,0x1A864DB2U,0x1E475005U,
        0x2608EDB8U,0x22C9F00FU,0x2F8AD6D6U,0x2B4BCB61U,
        0x350C9B64U,0x31CD86D3U,0x3C8EA00AU,0x384FBDBDU,
        0x4C11DB70U,0x48D0C6C7U,0x4593E01EU,0x4152FDA9U,
        0x5F15ADACU,0x5BD4B01BU,0x569796C2U,0x52568B75U,
        0x6A1936C8U,0x6ED82B7FU,0x639B0DA6U,0x675A1011U,
        0x791D4014U,0x7DDC5DA3U,0x709F7B7AU,0x745E66CDU,
        0x9823B6E0U,0x9CE2AB57U,0x91A18D8EU,0x95609039U,
        0x8B27C03CU,0x8FE6DD8BU,0x82A5FB52U,0x8664E6E5U,
        0xBE2B5B58U,0xBAEA46EFU,0xB7A96036U,0xB3687D81U,
        0xAD2F2D84U,0xA9EE3033U,0xA4AD16EAU,0xA06C0B5DU,
        0xD4326D90U,0xD0F37027U,0xDDB056FEU,0xD9714B49U,
        0xC7361B4CU,0xC3F706FBU,0xCEB42022U,0xCA753D95U,
        0xF23A8028U,0xF6FB9D9FU,0xFBB8BB46U,0xFF79A6F1U,
        0xE13EF6F4U,0xE5FFEB43U,0xE8BCCD9AU,0xEC7DD02DU,
        0x34867077U,0x30476DC0U,0x3D044B19U,0x39C556AEU,
        0x278206ABU,0x23431B1CU,0x2E003DC5U,0x2AC12072U,
        0x128E9DCFU,0x164F8078U,0x1B0CA6A1U,0x1FCDBB16U,
        0x018AEB13U,0x054BF6A4U,0x0808D07DU,0x0CC9CDCAU,
        0x7897AB07U,0x7C56B6B0U,0x71159069U,0x75D48DDEU,
        0x6B93DDDBU,0x6F52C06CU,0x6211E6B5U,0x66D0FB02U,
        0x5E9F46BFU,0x5A5E5B08U,0x571D7DD1U,0x53DC6066U,
        0x4D9B3063U,0x495A2DD4U,0x44190B0DU,0x40D816BAU,
        0xACA5C697U,0xA864DB20U,0xA527FDF9U,0xA1E6E04EU,
        0xBFA1B04BU,0xBB60ADFCU,0xB6238B25U,0xB2E29692U,
        0x8AAD2B2FU,0x8E6C3698U,0x832F1041U,0x87EE0DF6U,
        0x99A95DF3U,0x9D684044U,0x902B669DU,0x94EA7B2AU,
        0xE0B41DE7U,0xE4750050U,0xE9362689U,0xEDF73B3EU,
        0xF3B06B3BU,0xF771768CU,0xFA325055U,0xFEF34DE2U,
        0xC6BCF05FU,0xC27DEDE8U,0xCF3ECB31U,0xCBFFD686U,
        0xD5B88683U,0xD1799B34U,0xDC3ABDEDU,0xD8FBA05AU,
        0x690CE0EEU,0x6DCDFD59U,0x608EDB80U,0x644FC637U,
        0x7A089632U,0x7EC98B85U,0x738AAD5CU,0x774BB0EBU,
        0x4F040D56U,0x4BC510E1U,0x46863638U,0x42472B8FU,
        0x5C007B8AU,0x58C1663DU,0x558240E4U,0x51435D53U,
        0x251D3B9EU,0x21DC2629U,0x2C9F00F0U,0x285E1D47U,
        0x36194D42U,0x32D850F5U,0x3F9B762CU,0x3B5A6B9BU,
        0x0315D626U,0x07D4CB91U,0x0A97ED48U,0x0E56F0FFU,
        0x1011A0FAU,0x14D0BD4DU,0x19939B94U,0x1D528623U,
        0xF12F560EU,0xF5EE4BB9U,0xF8AD6D60U,0xFC6C70D7U,
        0xE22B20D2U,0xE6EA3D65U,0xEBA91BBCU,0xEF68060BU,
        0xD727BBB6U,0xD3E6A601U,0xDEA580D8U,0xDA649D6FU,
        0xC423CD6AU,0xC0E2D0DDU,0xCDA1F604U,0xC960EBB3U,
        0xBD3E8D7EU,0xB9FF90C9U,0xB4BCB610U,0xB07DABA7U,
        0xAE3AFBA2U,0xAAFBE615U,0xA7B8C0CCU,0xA379DD7BU,
        0x9B3660C6U,0x9FF77D71U,0x92B45BA8U,0x9675461FU,
        0x8832161AU,0x8CF30BADU,0x81B02D74U,0x857130C3U,
        0x5D8A9099U,0x594B8D2EU,0x5408ABF7U,0x50C9B640U,
        0x4E8EE645U,0x4A4FFBF2U,0x470CDD2BU,0x43CDC09CU,
        0x7B827D21U,0x7F436096U,0x7200464FU,0x76C15BF8U,
        0x68860BFDU,0x6C47164AU,0x61043093U,0x65C52D24U,
        0x119B4BE9U,0x155A565EU,0x18197087U,0x1CD86D30U,
        0x029F3D35U,0x065E2082U,0x0B1D065BU,0x0FDC1BECU,
        0x3793A651U,0x3352BBE6U,0x3E119D3FU,0x3AD08088U,
        0x2497D08DU,0x2056CD3AU,0x2D15EBE3U,0x29D4F654U,
        0xC5A92679U,0xC1683BCEU,0xCC2B1D17U,0xC8EA00A0U,
        0xD6AD50A5U,0xD26C4D12U,0xDF2F6BCBU,0xDBEE767CU,
        0xE3A1CBC1U,0xE760D676U,0xEA23F0AFU,0xEEE2ED18U,
        0xF0A5BD1DU,0xF464A0AAU,0xF9278673U,0xFDE69BC4U,
        0x89B8FD09U,0x8D79E0BEU,0x803AC667U,0x84FBDBD0U,
        0x9ABC8BD5U,0x9E7D9662U,0x933EB0BBU,0x97FFAD0CU,
        0xAFB010B1U,0xAB710D06U,0xA6322BDFU,0xA2F33668U,
        0xBCB4666DU,0xB8757BDAU,0xB5365D03U,0xB1F740B4U,
    };
#endif

    while (len > 0)
    {
#ifdef CRC_TABLE
        crc = crc_table[*data ^ ((crc >> 24) & 0xff)] ^ (crc << 8);
#else
        crc = crc_table(*data ^ ((crc >> 24) & 0xff)) ^ (crc << 8);
#endif
        data++;
        len--;
    }
    return crc;
}

uint32_t CRC32(const uint8_t *buf, uint8_t len)
{
    return digital_update_crc32(0xffffffff, buf, len) ^ 0xffffffff;
}


uint16_t NBFi_Phy_To_Bitrate(nbfi_phy_channel_t ch)
{
    switch(ch)
    {
    case DL_PSK_200:
    case UL_PSK_200:
        return 200;
    case DL_PSK_500:
    case UL_PSK_500:
        return 500;
    case DL_PSK_5000:
    case UL_PSK_5000:
        return 5000;
    case DL_PSK_FASTDL:
    case UL_PSK_FASTDL:
        return 57600;
//    case UL_DBPSK_50_PROT_C:
    case UL_DBPSK_50_PROT_D:
//    case UL_DBPSK_50_PROT_E:
        return 50;
//    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
//    case UL_DBPSK_400_PROT_E:
        return 400;
    case UL_DBPSK_3200_PROT_D:
//    case UL_DBPSK_3200_PROT_E:
        return 3200;
    case UL_DBPSK_25600_PROT_D:
//    case UL_DBPSK_25600_PROT_E:
        return 25600;
    }
    return 0;
}

uint8_t NBFi_Get_TX_Iter()
{
    return nbfi_state.UL_iter&0x1f;
}

static uint32_t delaymstimer;
void delay_ms(uint16_t ms)
{
    // scaling: 20e6/64/1e3=312.5=2^8+2^6-2^3+2^-1
    uint32_t x;

    x = ms;
    delaymstimer = ms >> 1;
    x <<= 3;
    delaymstimer -= x;
    x <<= 3;
    delaymstimer += x;
    x <<= 2;
    delaymstimer += x;

    do {
        delaymstimer--;
    }
    while (delaymstimer);
}
