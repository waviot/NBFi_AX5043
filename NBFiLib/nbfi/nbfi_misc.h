#ifndef NBFI_MISC_H
#define NBFI_MISC_H

typedef enum
{
    PACKET_FREE             = 0,
    PACKET_ALLOCATED        = 1,
    PACKET_QUEUED           = 2,
    PACKET_QUEUED_AGAIN     = 3,
    PACKET_TX               = 4,
    PACKET_SENT             = 5,
    PACKET_SENT_NOACKED     = 6,
    PACKET_RECEIVED         = 7,
    PACKET_DELIVERED        = 8,
    PACKET_LOST             = 9,
    PACKET_PROCESSING       = 0x0A,
    PACKET_PROCESSED        = 0x0B,
    PACKET_SKIPPED          = 0x0C,
    PACKET_REC_AND_SENT     = 0x0D,
    PACKET_WAIT_ACK         = 0x10,
    PACKET_GOT_DL           = 0x11,
    PACKET_DL_TIMEOUT       = 0x12,
    PACKET_NEED_TO_SEND_RIGHT_NOW   = 0x13,
    PACKET_WAIT_FOR_EXTRA_PACKETS   = 0x14,
    PACKET_CLEARED          = 0x15
}nbfi_packet_state_t;

/*NBFi physical layer packet struct*/
typedef struct
{
    union
    {
        struct
        {
            uint8_t ITER        : 5;//LSB
            uint8_t MULTI       : 1;
            uint8_t ACK         : 1;
            uint8_t SYS         : 1;//MSB
        };
        uint8_t header;
    };
    uint8_t payload[0];     //begining of packet payload
}nbfi_pfy_packet_t;


/*NBFi transport layer packet struct*/
typedef struct
{
    nbfi_packet_state_t state;              //packet state
    nbfi_handshake_t    handshake;          //packet handshake mode
    uint8_t             retry_num;          //retry counter
    uint8_t             mack_num;           //number of packets for multi ack mode
    uint8_t             phy_data_length;    //length of packet payload(without header)
    nbfi_pfy_packet_t   phy_data;           //physical layer packet data
}nbfi_transport_packet_t;


nbfi_transport_packet_t*            NBFi_AllocateTxPkt(uint8_t payload_length);
void                                NBFi_TxPacket_Free(nbfi_transport_packet_t* pkt);
void                                NBFi_RxPacket_Free(nbfi_transport_packet_t* pkt);
nbfi_transport_packet_t*            NBFi_AllocateRxPkt(uint8_t header, uint8_t payload_length);
nbfi_transport_packet_t*            NBFi_GetQueuedTXPkt();
void                                NBFi_Mark_Lost_All_Unacked();
_Bool                               NBFi_Check_RX_Packet_Duplicate(nbfi_pfy_packet_t * pkt, uint8_t len);
nbfi_transport_packet_t*            NBFi_Get_QueuedRXPkt(uint8_t *groupe, uint16_t *total_length);
nbfi_transport_packet_t*            NBFi_GetSentTXPkt_By_Iter(uint8_t iter);
uint8_t                             NBFi_Calc_Queued_Sys_Packets_With_Type(uint8_t type);
uint8_t                             NBFi_Calc_Packets_With_State(uint8_t state);
uint32_t                            NBFi_Get_RX_ACK_Mask();
void                                NBFi_Resend_Pkt(nbfi_transport_packet_t* act_pkt, uint32_t mask);
void                                NBFi_Clear_RX_Buffer();
void                                NBFi_Clear_TX_Buffer();
void                                NBFi_Send_Clear_Cmd(uint8_t iter);
uint16_t                            CRC16(uint8_t *buf, uint16_t len, uint16_t crc);
uint8_t                             CRC8(uint8_t* data, uint8_t len);
uint32_t                            CRC32(const uint8_t *buf, uint8_t len);
uint16_t                            NBFi_Phy_To_Bitrate(nbfi_phy_channel_t ch);
uint8_t                             NBFi_Get_TX_Iter();
void 				    delay_ms(uint16_t ms);

#endif