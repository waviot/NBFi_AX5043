#include "nbfi.h"
#include "nbfi_phy.h"
#include "nbfi_config.h"
#include "nbfi_misc.h"
#include "nbfi_defines.h"
#include "rf.h"
#include "xtea.h"
#include <stdlib.h>
#include <wtimer.h>
#include <string.h>

#define memset_xdata memset
#define memcpy_xdata memcpy
#define memcpy_xdatageneric memcpy
#define memcpy_genericxdata memcpy

extern nbfi_transport_packet_t * nbfi_TX_pktBuf[NBFI_TX_PKTBUF_SIZE];
extern nbfi_transport_packet_t* nbfi_RX_pktBuf[NBFI_RX_PKTBUF_SIZE];


nbfi_state_t nbfi_state = {0,0,0,0,-150,0,0,0,0,0,0,0,0};

extern nbfi_dev_info_t dev_info;

#ifdef FORMAT_CODE
#pragma default_function_attributes = @ "NBFi_FUNC"
#endif

const uint32_t NBFI_DL_DELAY[10] = {MILLISECONDS(30000), MILLISECONDS(30000), MILLISECONDS(30000), MILLISECONDS(5000), MILLISECONDS(5000), MILLISECONDS(5000), MILLISECONDS(1000), MILLISECONDS(1000), MILLISECONDS(500), MILLISECONDS(500)};
const uint32_t NBFI_DL_LISTEN_TIME[4] = {MILLISECONDS(40000), MILLISECONDS(40000), MILLISECONDS(40000), MILLISECONDS(40000)};
const uint32_t NBFI_DL_ADD_RND_LISTEN_TIME[4] = {MILLISECONDS(20000), MILLISECONDS(20000), MILLISECONDS(20000), MILLISECONDS(20000)};

#define DRXLISTENAFTERSEND  20

#define WAITALITTLEBIT  3000

const uint8_t NBFI_NOISE_DINAMIC[4] = {20, 8, 5, 5};

nbfi_transport_packet_t idle_pkt = {PACKET_FREE, HANDSHAKE_NONE, 0, 0, 0, {0,0} };
nbfi_transport_packet_t* nbfi_active_pkt = &idle_pkt;
nbfi_packet_state_t nbfi_active_pkt_old_state;

struct wtimer_desc nbfi_processTask_desc;
struct wtimer_desc dl_receive_desc;
struct wtimer_desc dl_drx_desc;
struct wtimer_desc wait_for_extra_desc;
struct wtimer_desc nbfi_heartbeat_desc;

int16_t rssi = 0;
int16_t offset = 0;

rx_handler_t  rx_handler = 0;

uint8_t not_acked = 0;


uint8_t aver_tx_snr = 0;
uint8_t aver_rx_snr = 0;
uint8_t nbfi_last_snr = 0;

int16_t noise_summ = 0;
uint8_t noise_cntr = 0;
int16_t noise_min = -150;
uint8_t noise_min_cntr = 2;

_Bool wait_Receive = 0;
_Bool wait_Extra = 0;
_Bool wait_RxEnd = 0;

_Bool rx_complete = 0;

uint32_t info_timer;

uint32_t MinVoltage = 0;

uint32_t nbfi_rtc = 0;

_Bool process_rx_external = 0;


static void    NBFi_Receive_Timeout_cb(struct wtimer_desc *desc);
static void    NBFi_RX_DL_EndHandler(struct wtimer_desc *desc);
static void    NBFi_Wait_Extra_Handler(struct wtimer_desc *desc);
static void    NBFi_SendHeartBeats(struct wtimer_desc *desc);
static void    NBFi_Force_process();

_Bool NBFi_Config_Tx_Power_Change(nbfi_rate_direct_t dir);
void NBFi_Config_Return();
_Bool NBFi_Config_Send_Mode(_Bool, uint8_t);
void NBFi_Config_Set_Default();
void NBFi_ReadConfig(nbfi_settings_t *settings);
void NBFi_WriteConfig();
void NBFi_Config_Set_TX_Chan(nbfi_phy_channel_t ch);
void NBFi_Config_Set_RX_Chan(nbfi_phy_channel_t ch);
void NBFi_Config_Set_Device_Info(nbfi_dev_info_t *);
void NBFI_Config_Check_State();

extern uint8_t you_should_dl_power_step_up;
extern uint8_t you_should_dl_power_step_down;
extern uint8_t current_tx_rate;
extern uint8_t current_rx_rate;

void (* __nbfi_on_off_pwr)(uint8_t) = 0;
void (* __nbfi_before_tx)(NBFi_ax5043_pins_s *) = 0;
void (* __nbfi_before_rx)(NBFi_ax5043_pins_s *) = 0;
void (* __nbfi_before_off)(NBFi_ax5043_pins_s *) = 0;
void (* __nbfi_read_default_settings)(nbfi_settings_t*) = 0;
void (* __nbfi_read_flash_settings)(nbfi_settings_t*)  = 0;
void (* __nbfi_write_flash_settings)(nbfi_settings_t*) = 0;
uint32_t (* __nbfi_measure_voltage_or_temperature)(uint8_t) = 0;
uint32_t (* __nbfi_update_rtc)(void) = 0;
void (* __nbfi_rtc_synchronized)(uint32_t) = 0;
void (* __nbfi_lock_unlock_nbfi_irq)(uint8_t) = 0;
void (* __nbfi_reset)(void) = 0;


void NBFI_reg_func(uint8_t name, void* fn)
{
	switch(name)
	{
        break;
	case NBFI_BEFORE_TX:
		__nbfi_before_tx = (void(*)(NBFi_ax5043_pins_s*))fn;
		break;
	case NBFI_BEFORE_RX:
		__nbfi_before_rx = (void(*)(NBFi_ax5043_pins_s*))fn;
		break;
        case NBFI_BEFORE_OFF:
		__nbfi_before_off = (void(*)(NBFi_ax5043_pins_s*))fn;
		break;
	case NBFI_RECEIVE_COMLETE:
		rx_handler = (rx_handler_t)fn;
		break;
	case NBFI_READ_DEFAULT_SETTINGS:
		__nbfi_read_default_settings = (void(*)(nbfi_settings_t*))fn;
		break;
	case NBFI_READ_FLASH_SETTINGS:
		__nbfi_read_flash_settings = (void(*)(nbfi_settings_t*))fn;
		break;
	case NBFI_WRITE_FLASH_SETTINGS:
		__nbfi_write_flash_settings = (void(*)(nbfi_settings_t*))fn;
		break;
	case NBFI_MEASURE_VOLTAGE_OR_TEMPERATURE:
		__nbfi_measure_voltage_or_temperature = (uint32_t(*)(uint8_t))fn;
		break;
        case NBFI_UPDATE_RTC:
            __nbfi_update_rtc = (uint32_t(*)(void))fn;
            break;
        case NBFI_RTC_SYNCHRONIZED:
            __nbfi_rtc_synchronized = (void(*)(uint32_t))fn;
            break;
        case NBFI_LOCKUNLOCKNBFIIRQ:
            __nbfi_lock_unlock_nbfi_irq = (void(*)(uint8_t))fn;
            break; 
        case NBFI_RESET:
            __nbfi_reset = (void(*)(void))fn;
            break;
	default:
		break;
	}
}


nbfi_status_t NBFi_Send(uint8_t* payload, uint8_t length)
{
    nbfi_transport_packet_t* packet;
    uint8_t groupe = 0;
    uint8_t len = length;
    
    if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(1);
    
    uint8_t free = NBFI_TX_PKTBUF_SIZE - NBFi_Packets_To_Send();
    if((length <= nbfi.max_payload_len) && (free < nbfi.mack_mode + 3 ) ) 
    {
      if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(0);
      return ERR_BUFFER_FULL;
    }
    else if((length/nbfi.max_payload_len + 3) > free) 
    {
      if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(0);
      return ERR_BUFFER_FULL;
    }
    if(length < nbfi.max_payload_len)
    {
        packet =  NBFi_AllocateTxPkt(length + 1);
        if(!packet)
        {
            if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(0);
            return ERR_BUFFER_FULL;
        }
        packet->phy_data.SYS = 1;
        packet->phy_data.payload[0] = 0x80 + (length & 0x7f);
        memcpy_xdata(&packet->phy_data.payload[1], (void const*)payload, length);
        packet->state = PACKET_QUEUED;
        packet->handshake = nbfi.handshake_mode;
        packet->phy_data.ITER = nbfi_state.UL_iter++ & 0x1f;
        if(nbfi.handshake_mode != HANDSHAKE_NONE)
        {
            if(((nbfi_state.UL_iter) % nbfi.mack_mode) == 0)
            {
                packet->phy_data.ACK = 1;
                packet->mack_num = not_acked + 1;
                not_acked = 0;
            }
            else not_acked++;
        }

    }
    else do
    {
        uint8_t l;
        uint8_t first = 0;
        if(length > nbfi.max_payload_len)
        {
            first = (groupe == 0)*3;
            l = nbfi.max_payload_len - first;
        }
        else l = length;
        packet =  NBFi_AllocateTxPkt(l + first);
        if(!packet)
        {
            if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(0);
            return ERR_BUFFER_FULL;
        }
        
        memcpy_xdata(packet->phy_data.payload + first, (void const*)&payload[groupe * nbfi.max_payload_len - 3*(groupe != 0)], l);
        packet->state = PACKET_QUEUED;
        packet->handshake = nbfi.handshake_mode;
        packet->phy_data.ITER = nbfi_state.UL_iter++ & 0x1f;
        if(l < length)
        {
            packet->phy_data.MULTI = 1;
            if(groupe == 0) //the start packet of the groupe must be system
            {
                packet->phy_data.SYS = 1;
                packet->phy_data.payload[0] = 0x02;
                packet->phy_data.payload[1] = len + 1;
                packet->phy_data.payload[2] = CRC8(payload, len);
            }
        }

        length -= l;
        groupe++;
        if((length == 0) && (groupe == 1))
        {
            if(nbfi.handshake_mode != HANDSHAKE_NONE)
            {
                if(((nbfi_state.UL_iter) % nbfi.mack_mode) == 0)
                {
                    packet->phy_data.ACK = 1;
                    packet->mack_num = not_acked + 1;
                    not_acked = 0;
                }
                else not_acked++;
            }
        }
        else   //the last packet of groupe must be acked
        {
            packet->phy_data.MULTI = 1;
            if(nbfi.handshake_mode != HANDSHAKE_NONE)
            {
                if(length == 0)
                {
                    packet->phy_data.ACK = 1;
                    packet->mack_num = groupe + not_acked;
                    not_acked = 0;
                }
            }
        }

    }while(length);
    
    
    NBFi_Force_process();
    if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(0);
    return OK;
}

void NBFi_ProcessRxPackets(_Bool external)
{
    nbfi_transport_packet_t* pkt;
    uint8_t data[256];
    uint8_t groupe;
    uint8_t last_group_iter;
    uint16_t total_length;
    _Bool group_with_crc = 0;
    process_rx_external = external;
    
    while(1)
    {

        if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(1);
        
        pkt = NBFi_Get_QueuedRXPkt(&groupe, &total_length);

        if(!pkt)    
        {
          if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(0);
          return;
        }

        if((pkt->phy_data.SYS) && (pkt->phy_data.payload[0] & 0x80))
        {
            total_length = pkt->phy_data.payload[0] & 0x7f;
            total_length = total_length%nbfi.max_payload_len;
            memcpy_xdata(data, (void const*)(&pkt->phy_data.payload[1]), total_length);
            if(nbfi.mack_mode < MACK_2) pkt->state = PACKET_PROCESSED;//NBFi_RxPacket_Free(pkt);
        }
        else
        {
            uint8_t iter = pkt->phy_data.ITER;
            group_with_crc = ((pkt->phy_data.SYS) && (nbfi_RX_pktBuf[(iter)&0x1f]->phy_data.payload[0] == 0x02));
            uint16_t memcpy_len = total_length;
            for(uint8_t i = 0; i != groupe; i++)
            {
                uint8_t len;
                uint8_t first = 0;
                last_group_iter = (iter + i)&0x1f;
                if((i == 0)&&(groupe > 1)) {len = nbfi.max_payload_len - 2; first = 2;}
                else len = (memcpy_len>=nbfi.max_payload_len)?nbfi.max_payload_len:memcpy_len%nbfi.max_payload_len;
                memcpy_xdata(data + i*nbfi.max_payload_len - 2*(i != 0), (void const*)(&nbfi_RX_pktBuf[last_group_iter]->phy_data.payload[first]), len);
                memcpy_len -= len;
                if(nbfi_RX_pktBuf[last_group_iter]->phy_data.ACK) nbfi_RX_pktBuf[last_group_iter]->state = PACKET_CLEARED;
                else nbfi_RX_pktBuf[last_group_iter]->state = PACKET_PROCESSED;

                if((nbfi.mack_mode < MACK_2) && (groupe == 1)) 
                {
                  //NBFi_RxPacket_Free(nbfi_RX_pktBuf[(iter + i)&0x1f]);
                  nbfi_RX_pktBuf[last_group_iter]->state = PACKET_PROCESSED;
                }

            }
        }
        
        
        uint8_t *data_ptr;
        if(group_with_crc)
        {
            total_length--;
            if(CRC8((unsigned char*)(&data[1]), (unsigned char)(total_length)) != data[0]) 
            {
                NBFi_Clear_RX_Buffer();
                if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(0);
                return;
            }
            data_ptr = &data[1];
        }
        else data_ptr = &data[0];
        
        if(groupe > 1) NBFi_Wait_Extra_Handler(0);
        
        if(__nbfi_lock_unlock_nbfi_irq) __nbfi_lock_unlock_nbfi_irq(0);
        
        if(rx_handler) rx_handler(data_ptr, total_length);
        
    }

}


void NBFi_ParseReceivedPacket(struct axradio_status *st)
{
    int16_t rtc_offset;

    if((nbfi.mode != TRANSPARENT) && (!NBFi_Match_ID((uint8_t *)st->u.rx.pktdata))) return;

    rx_complete = 1;

    
#ifndef NOKEYDL
    if(!(nbfi.additional_flags&NBFI_FLG_NO_XTEA))
    {
        st->u.rx.pktlen -= 2;
        if(XTEA_Enabled() && XTEA_Available())
        {
        NBFi_XTEA_OFB((uint8_t*)(st->u.rx.pktdata + 4), st->u.rx.pktlen - 5 - 3, st->u.rx.pktdata[3]&0x1f);
        }
        uint16_t crc_calc = CRC16((uint8_t *)(st->u.rx.pktdata + 4), st->u.rx.pktlen - 5 - 3, 0xFFFF);
        uint16_t crc_rx = st->u.rx.pktdata[st->u.rx.pktlen - 3];
        crc_rx <<= 8;
        crc_rx |= st->u.rx.pktdata[st->u.rx.pktlen - 4];
        if(crc_rx != crc_calc) return;
    }
#endif

    nbfi_state.DL_total++;
    nbfi_state.DL_last_time = NBFi_get_RTC();
    noise_min_cntr =  NBFI_NOISE_DINAMIC[nbfi.rx_phy_channel];
    uint8_t snr;
    if(st->u.rx.phy.rssi < nbfi_state.noise) snr = 0;
    else snr = (st->u.rx.phy.rssi - nbfi_state.noise) & 0xff;

    nbfi_state.last_rssi = st->u.rx.phy.rssi;
    
    if(snr > 5)
    {
      nbfi_last_snr = snr;
      nbfi_state.aver_rx_snr = (((uint16_t)nbfi_state.aver_rx_snr)*3 + snr)>>2;
    }

    nbfi_transport_packet_t* pkt = 0;

    if(nbfi.mode == TRANSPARENT)
    {
        NBFi_RX_Controller();
        return;
    }


    nbfi_pfy_packet_t *phy_pkt = (nbfi_pfy_packet_t *)st->u.rx.pktdata + 3;


    wtimer0_remove(&wait_for_extra_desc);
    wait_Extra = 0;

    if(nbfi_active_pkt->state == PACKET_WAIT_FOR_EXTRA_PACKETS)
    {
        nbfi_active_pkt->state = nbfi_active_pkt_old_state;
    }

    uint32_t mask = 0;
    uint8_t i = 1;
    uint32_t rtc;
    if(phy_pkt->SYS)
    {
            /* System messages */
            if(phy_pkt->payload[0] & 0x80) goto place_to_stack;
            switch(phy_pkt->payload[0]) // Message type
            {
            case 0x00:  //ACK received

                if(((nbfi_active_pkt->state == PACKET_WAIT_ACK) ) && (phy_pkt->ITER == nbfi_active_pkt->phy_data.ITER))
                {
                    wtimer0_remove(&dl_receive_desc);
                    wait_Receive = 0;

                    if(nbfi_active_pkt->mack_num == 0)
                    {
                        nbfi_state.success_total++;
                        nbfi_active_pkt->state =  PACKET_DELIVERED;
                    }
                    nbfi_state.aver_tx_snr = (((uint16_t)nbfi_state.aver_tx_snr)*3 + phy_pkt->payload[5])>>2;
                    nbfi_station_info.info = phy_pkt->payload[7];
                    
                    if(nbfi_station_info.RTC_MSB&0x20) rtc_offset = 0xC0 | nbfi_station_info.RTC_MSB;
                    else rtc_offset = nbfi_station_info.RTC_MSB;
                    rtc_offset <<= 8;
                    rtc_offset |= phy_pkt->payload[6];
                    if(rtc_offset) NBFi_set_RTC(NBFi_get_RTC() + rtc_offset);
                    
                    do
                    {
                        mask = (mask << 8) + phy_pkt->payload[i];
                    }   while (++i < 5);

                    NBFi_Resend_Pkt(nbfi_active_pkt, mask);

                }
                break;
                
            case 03:    //ACK on system packet received
                if((nbfi_active_pkt->state == PACKET_WAIT_ACK))
                {
                    wtimer0_remove(&dl_receive_desc);
                    wait_Receive = 0;
                    nbfi_active_pkt->state =  PACKET_DELIVERED;
                    nbfi_state.success_total++;
                }
                break;
            case 0x04:  //clear RX buffer message received
                NBFi_Clear_RX_Buffer();
                break;
            case 0x02:  //start packet of the groupe
            case 0x05:  
                goto place_to_stack;
            case 0x06:  //nbfi configure

                if(NBFi_Config_Parser(&phy_pkt->payload[1]))
                {
                    nbfi_transport_packet_t* ack_pkt =  NBFi_AllocateTxPkt(8);
                    if(!ack_pkt) break;
                    ack_pkt->phy_data.payload[0] = 0x06;
                    memcpy_xdata(&ack_pkt->phy_data.payload[1], &phy_pkt->payload[1], 7);
                    ack_pkt->phy_data.ITER = phy_pkt->ITER;
                    ack_pkt->phy_data.header |= SYS_FLAG;
                    ack_pkt->state = PACKET_NEED_TO_SEND_RIGHT_NOW;
                }
                break;
            case 0x07: //software reset
                if(__nbfi_reset && (phy_pkt->payload[1] == 0xDE) && (phy_pkt->payload[2] == 0xAD)) __nbfi_reset();
                break;
            case 0x09:  //time correction
              memcpy(&rtc, &phy_pkt->payload[1], 4);
              NBFi_set_RTC(rtc);
              break;
            }
            if(phy_pkt->ACK && !NBFi_Calc_Queued_Sys_Packets_With_Type(0))    //send ACK on system packet
            {
                    nbfi_transport_packet_t* ack_pkt =  NBFi_AllocateTxPkt(8);
                    if(ack_pkt)
                    {

                        ack_pkt->phy_data.payload[0] = 0x03; //ACK on SYS
                        ack_pkt->phy_data.payload[1] = phy_pkt->payload[0];//type of sys packet
                        ack_pkt->phy_data.payload[2] = 0;
                        ack_pkt->phy_data.payload[3] = 0;
                        ack_pkt->phy_data.payload[4] = 0;
                        ack_pkt->phy_data.payload[5] = snr;
                        ack_pkt->phy_data.payload[6] = (uint8_t)(nbfi_state.noise + 150);
                        ack_pkt->phy_data.payload[7] = you_should_dl_power_step_down + you_should_dl_power_step_up + (nbfi.tx_pwr & 0x3f); 
                        ack_pkt->phy_data.ITER = phy_pkt->ITER;
                        ack_pkt->phy_data.header |= SYS_FLAG;
                        ack_pkt->handshake = HANDSHAKE_NONE;
                        ack_pkt->state = PACKET_NEED_TO_SEND_RIGHT_NOW;
                    }
            }

    }
    else
    {
        //Get application packet
place_to_stack:
        if(!NBFi_Check_RX_Packet_Duplicate(phy_pkt, st->u.rx.pktlen - 4 - 3))   //if there is no rx duplicate
        {
            pkt = NBFi_AllocateRxPkt(phy_pkt->header, st->u.rx.pktlen - 5 - 3);
            if(!pkt) return;
            memcpy_xdata(&pkt->phy_data.header, phy_pkt, st->u.rx.pktlen - 4 - 3);
            pkt->state = PACKET_RECEIVED;
        }

        if(phy_pkt->ACK && !NBFi_Calc_Queued_Sys_Packets_With_Type(0))
        {
            // Send ACK
            nbfi_transport_packet_t* ack_pkt =  NBFi_AllocateTxPkt(8);
            if(ack_pkt)
            {
                uint32_t mask = NBFi_Get_RX_ACK_Mask();
                ack_pkt->phy_data.payload[0] = 0x00;
                ack_pkt->phy_data.payload[1] = (mask >> 24)&0xff;
                ack_pkt->phy_data.payload[2] = (mask >> 16)&0xff;
                ack_pkt->phy_data.payload[3] = (mask >> 8)&0xff;
                ack_pkt->phy_data.payload[4] = (mask >> 0)&0xff;
                ack_pkt->phy_data.payload[5] = snr;
                ack_pkt->phy_data.payload[6] = (uint8_t)(nbfi_state.noise + 150);
                ack_pkt->phy_data.payload[7] = you_should_dl_power_step_down + you_should_dl_power_step_up + (nbfi.tx_pwr & 0x3f); 
                ack_pkt->phy_data.ITER = nbfi_state.DL_iter&0x1f;
                ack_pkt->phy_data.header |= SYS_FLAG;
                ack_pkt->handshake = HANDSHAKE_NONE;
                ack_pkt->state = PACKET_NEED_TO_SEND_RIGHT_NOW;
                NBFi_Force_process();
            }
        }
    }

    if(phy_pkt->MULTI && !phy_pkt->ACK)
    {
        //wait for extra packets
        if(nbfi_active_pkt->state != PACKET_WAIT_FOR_EXTRA_PACKETS)
        {
          nbfi_active_pkt_old_state = nbfi_active_pkt->state;
          nbfi_active_pkt->state = PACKET_WAIT_FOR_EXTRA_PACKETS;
        }
        ScheduleTask(&wait_for_extra_desc, NBFi_Wait_Extra_Handler, RELATIVE, NBFI_DL_LISTEN_TIME[nbfi.rx_phy_channel]);
        wait_Extra = 1;
    }
    else
    {
        if(nbfi_active_pkt->state == PACKET_WAIT_FOR_EXTRA_PACKETS) nbfi_active_pkt->state = nbfi_active_pkt_old_state;

    }
    
    if(process_rx_external == 0) NBFi_ProcessRxPackets(0);
        
    if(!phy_pkt->ACK) NBFI_Config_Check_State();
    if(NBFi_GetQueuedTXPkt()) NBFi_Force_process();
    else
    {
        if(nbfi.mode == DRX)
        {
            wait_RxEnd = 1;
            ScheduleTask(&dl_drx_desc, NBFi_RX_DL_EndHandler, RELATIVE, MILLISECONDS(WAITALITTLEBIT));
        }
        
        NBFi_RX_Controller();
    }

}


static void NBFi_ProcessTasks(struct wtimer_desc *desc)
{
   nbfi_transport_packet_t* pkt;
   static _Bool need_to_calc_noise = 0;
   static uint16_t cal_noise_timer = 20*60*5;
   if(nbfi.mode == OFF)
   {
        NBFi_RX_Controller();
        NBFi_Clear_TX_Buffer();
        ScheduleTask(desc, 0, RELATIVE, SECONDS(30));
        return;
   }
   if((rf_busy == 0)&&(transmit == 0)&&!((need_to_calc_noise == 1)&&(rf_state == STATE_RX)))
   {
        switch(nbfi_active_pkt->state)
        {
        case PACKET_WAIT_ACK:
            if(!wait_Receive)
            {
                ScheduleTask(&dl_receive_desc, NBFi_Receive_Timeout_cb, RELATIVE, NBFI_DL_DELAY[nbfi.tx_phy_channel - 20] + NBFI_DL_LISTEN_TIME[nbfi.rx_phy_channel] + rand()%(NBFI_DL_ADD_RND_LISTEN_TIME[nbfi.rx_phy_channel]));
                wait_Receive = 1;
            }
            break;
        case PACKET_WAIT_FOR_EXTRA_PACKETS:
            if(!wait_Extra)
            {
                ScheduleTask(&wait_for_extra_desc, NBFi_Wait_Extra_Handler, RELATIVE, NBFI_DL_LISTEN_TIME[nbfi.rx_phy_channel]);
                wait_Extra = 1;
            }
            break;
        default:

            pkt = NBFi_GetQueuedTXPkt();
            if(pkt)
            {
                if((pkt->handshake != HANDSHAKE_NONE) && (nbfi.mode != TRANSPARENT))
                {
                    if(pkt->phy_data.ACK)
                    {
                        switch(nbfi.mode)
                        {
                        case DRX:
                        case CRX:
                            pkt->state = PACKET_WAIT_ACK;
                            ScheduleTask(&dl_receive_desc, NBFi_Receive_Timeout_cb, RELATIVE, NBFI_DL_DELAY[nbfi.tx_phy_channel - 20] + NBFI_DL_LISTEN_TIME[nbfi.rx_phy_channel] + rand()%(NBFI_DL_ADD_RND_LISTEN_TIME[nbfi.rx_phy_channel]));
                            wait_Receive = 1;
                            break;
                        case NRX:
                            pkt->state = PACKET_SENT;
                            break;
                        }
                    }else pkt->state = PACKET_SENT_NOACKED;
                }
                else pkt->state = PACKET_SENT;
                nbfi_active_pkt = pkt;
                if(/*pkt->phy_data.SYS && */!pkt->phy_data.ACK && NBFi_GetQueuedTXPkt()) pkt->phy_data.header |= MULTI_FLAG;

                if(pkt->phy_data.SYS && (pkt->phy_data.payload[0] == 0x08))
                {
                  uint32_t rtc = NBFi_get_RTC();
                  pkt->phy_data.SYS = 1;
                  memcpy(&pkt->phy_data.payload[1], &rtc, 4);
                }

                if(wait_RxEnd) {wait_RxEnd = 0; wtimer0_remove(&dl_drx_desc);}
                NBFi_TX(pkt);

                if(pkt->state == PACKET_SENT)
                {
                    NBFi_TxPacket_Free(pkt);
                    nbfi_active_pkt = &idle_pkt;
                }

            }
            else
            {
                    NBFi_RX_Controller();
            }
        }
    }
    else
    {
          uint32_t t = __nbfi_measure_voltage_or_temperature(1);
          if(t < MinVoltage || !MinVoltage) MinVoltage = t;
    }

    //if(need_to_calc_noise && (rf_state == STATE_CHANGED)) NBFi_RX_Controller();
    if(rf_state == STATE_RX)
    {
        if(++cal_noise_timer > 20*60*5) need_to_calc_noise = 1;
        if(noise_cntr >= 10)
        {
            int16_t n = noise_summ/noise_cntr;
            noise_summ = 0;
            noise_cntr = 0;
            if(noise_min == -150) noise_min = n;
            if(n < noise_min) noise_min = n;
            if(--noise_min_cntr == 0)
            {
                if(noise_min < -150) noise_min = -149;
                else nbfi_state.noise = noise_min;
                need_to_calc_noise = 0;
                cal_noise_timer = 0;
                noise_min = 0;
                noise_min_cntr =  NBFI_NOISE_DINAMIC[nbfi.rx_phy_channel];
            }

        }
        else
        {
            int8_t r = ax5043_spi_read(AX5043_RSSI);
            noise_summ += (nbfi_state.rssi = (r - (int16_t)axradio_phy_rssioffset));
            noise_cntr++;

        }
    }
    else noise_min_cntr =  NBFI_NOISE_DINAMIC[nbfi.rx_phy_channel];
   
   if(rf_state == STATE_CHANGED)  NBFi_RX_Controller();
   
    if(nbfi.mode <= DRX && !NBFi_GetQueuedTXPkt() && (rf_busy == 0) && (transmit == 0) )
    {
        NBFi_RX_Controller();
        if(rf_state == STATE_OFF) ScheduleTask(desc, 0, RELATIVE, SECONDS(10));
        else ScheduleTask(desc, 0, RELATIVE, MILLISECONDS(50));
    }
    else ScheduleTask(desc, 0, RELATIVE, MILLISECONDS(50));

}

void NBFi_TX_Finished()
{
    if(!nbfi_active_pkt->phy_data.ACK && NBFi_GetQueuedTXPkt())
    {
        NBFi_Force_process();
    }
    else
    {
        if(!nbfi_active_pkt->phy_data.ACK && (nbfi.mode == DRX))
        {
            wait_RxEnd = 1;
            ScheduleTask(&dl_drx_desc, NBFi_RX_DL_EndHandler, RELATIVE, SECONDS(DRXLISTENAFTERSEND));
        }
        else NBFI_Config_Check_State();
        NBFi_RX_Controller();
    }
}

static void NBFi_RX_DL_EndHandler(struct wtimer_desc *desc)
{
    wait_RxEnd = 0;
    NBFi_RX_Controller();
}


static void NBFi_Receive_Timeout_cb(struct wtimer_desc *desc)
{
    if(rf_busy)
    {
        ScheduleTask(desc, NBFi_Receive_Timeout_cb, RELATIVE, NBFI_DL_LISTEN_TIME[nbfi.rx_phy_channel]);
        return;
    }
    wtimer0_remove(&dl_receive_desc);
    wait_Receive = 0;
    if(nbfi_active_pkt->state != PACKET_WAIT_ACK)
    {
        #ifdef NBFi_DEBUG
                    my_sprintf((char *)string, "nbfi_active_pkt->state != PACKET_WAIT_ACK");
                    SLIP_Send_debug((uint8_t *)string, 50);
        #endif
        return;
    }
    nbfi_state.fault_total++;
    NBFi_Config_Tx_Power_Change(UP);
    if(++nbfi_active_pkt->retry_num > nbfi.num_of_retries)
    {
       nbfi_active_pkt->state = PACKET_LOST;

        if(nbfi_active_pkt->phy_data.SYS && (nbfi_active_pkt->phy_data.payload[0] == 0x06)/* && (nbfi_active_pkt->phy_data.payload[1] != RATE_CHANGE_PARAM_CMD)*/)
        {
            NBFi_Mark_Lost_All_Unacked();
            NBFi_Config_Return(); //return to previous work configuration
        }
        else
        {
            if(!(nbfi.additional_flags&NBFI_FLG_NO_RESET_TO_DEFAULTS))
            {
                if((current_tx_rate == 0)&&(current_rx_rate == 0))
                {

                    NBFi_Mark_Lost_All_Unacked();

                }
                else
                {
                    nbfi_active_pkt->retry_num = 0;
                    nbfi_active_pkt->state = PACKET_QUEUED;
                }
                NBFi_Config_Set_Default(); //set default configuration
                NBFi_Config_Send_Mode(0, NBFI_PARAM_MODE);
            }

        }
    }
    else
    {
        nbfi_active_pkt->state = PACKET_QUEUED_AGAIN;
    }
    NBFi_Force_process();
    return;
}

static void NBFi_Wait_Extra_Handler(struct wtimer_desc *desc)
{
    wtimer0_remove(&wait_for_extra_desc);
    wait_Extra = 0;
    if(nbfi_active_pkt->state == PACKET_WAIT_FOR_EXTRA_PACKETS)     {nbfi_active_pkt->state = nbfi_active_pkt_old_state;}
    if(NBFi_GetQueuedTXPkt()) NBFi_Force_process();
}



static void NBFi_update_RTC()
{
    static uint32_t old_time_cur = 0;
 
    if(__nbfi_update_rtc) 
    {
      nbfi_rtc = __nbfi_update_rtc();
      return;
    }
    
    uint32_t delta;
    
    uint32_t tmp = (wtimer_state[0].time.cur >> 10);

    if(old_time_cur <= tmp)
    {
        delta = tmp - old_time_cur;
    }
    else delta = old_time_cur - tmp;

    nbfi_rtc += delta;

    old_time_cur = tmp;
}

uint32_t NBFi_get_RTC()
{
    NBFi_update_RTC();
    return nbfi_rtc;
}

void NBFi_set_RTC(uint32_t time)
{
   NBFi_update_RTC();
   nbfi_rtc = time;
   if(__nbfi_rtc_synchronized) __nbfi_rtc_synchronized(nbfi_rtc);
}


static void NBFi_SendHeartBeats(struct wtimer_desc *desc)
{

    static uint16_t hb_timer = 0;
    static uint16_t osccal_timer = 0;

    NBFi_update_RTC();

    if(hb_timer == 0) hb_timer = rand()%nbfi.heartbeat_interval;

    if(nbfi.mode == OFF) 
    {
      ScheduleTask(&nbfi_heartbeat_desc, NBFi_SendHeartBeats, RELATIVE, SECONDS(60));
      return;
    }

    if(nbfi.mode <= DRX)
    {
        ScheduleTask(&nbfi_heartbeat_desc, NBFi_SendHeartBeats, RELATIVE, SECONDS(60));
    }
    else ScheduleTask(&nbfi_heartbeat_desc, NBFi_SendHeartBeats, RELATIVE, SECONDS(1));


    if(++hb_timer >= nbfi.heartbeat_interval + 1)
    {
        hb_timer = 1;
        if(nbfi.heartbeat_num == 0) return;
        if(nbfi.heartbeat_num != 0xff) nbfi.heartbeat_num--;
        if(NBFi_Calc_Queued_Sys_Packets_With_Type(1)) return;
        nbfi_transport_packet_t* ack_pkt =  NBFi_AllocateTxPkt(8);
        if(!ack_pkt)   return;
        ack_pkt->phy_data.payload[0] = 0x01;
        ack_pkt->phy_data.payload[1] = 0;                      //heart beat type
        ack_pkt->phy_data.payload[2] = (MinVoltage >= 300 ? 0x80 : 0) + MinVoltage % 100;         //min supply voltage since last heartbeat
        MinVoltage = 0; //reset min voltage detection
        ack_pkt->phy_data.payload[3] = __nbfi_measure_voltage_or_temperature(0);    //temperature
        ack_pkt->phy_data.payload[4] = nbfi_state.aver_rx_snr; // DL average snr
        ack_pkt->phy_data.payload[5] = nbfi_state.aver_tx_snr; // UL average snr
        ack_pkt->phy_data.payload[6] = (uint8_t)(nbfi_state.noise + 150); // rx noice
        ack_pkt->phy_data.payload[7] = nbfi.tx_pwr;            // output power
        ack_pkt->phy_data.ITER = nbfi_state.UL_iter++ & 0x1f;;
        ack_pkt->phy_data.header |= SYS_FLAG;
        if(nbfi.mode != NRX)
        {
            if(nbfi.handshake_mode != HANDSHAKE_NONE)  ack_pkt->handshake = HANDSHAKE_SIMPLE;
            ack_pkt->phy_data.header |= ACK_FLAG;
        }
        ack_pkt->state = PACKET_QUEUED;
    }

    if(!(nbfi.additional_flags&NBFI_FLG_NO_SENDINFO))
    {
        if(nbfi.mode <= DRX) info_timer += 60;
        else info_timer++;
        if(info_timer >= dev_info.send_info_interval)
        {
                info_timer = 0;
                NBFi_Config_Send_Mode(0, NBFI_PARAM_TX_BRATES);
                NBFi_Config_Send_Mode(0, NBFI_PARAM_RX_BRATES);
                NBFi_Config_Send_Mode(0, NBFI_PARAM_VERSION);
        }
    }


    if((nbfi.additional_flags&NBFI_FLG_DO_OSCCAL)&&(nbfi.mode <= DRX))
    {
        if((osccal_timer++%MAKE_OSCCAL_INTERVAL) == 0)
        {
            if(!rf_busy)
            {
                RF_Init(OSC_CAL, (rf_antenna_t)0, 0, 0);
            }
        }
    }

}

static void NBFi_Force_process()
{
    ScheduleTask(&nbfi_processTask_desc, NBFi_ProcessTasks, RELATIVE, MILLISECONDS(5));
}

nbfi_state_t* NBFi_get_state()
{
    return &nbfi_state;
}

void NBFi_Go_To_Sleep(_Bool sleep)
{
    static _Bool old_state = 1;
    if(sleep)
    {
        nbfi.mode = OFF;
        NBFi_Clear_TX_Buffer();
        RF_Deinit();
    }
    else
    {
        if(old_state != sleep)
        {
            nbfi_settings_t settings;
            NBFi_ReadConfig(&settings);
            nbfi.mode = settings.mode;
            NBFi_Config_Send_Mode(0, NBFI_PARAM_MODE);
            NBFi_Send_Clear_Cmd(0);
            NBFi_Force_process();
        }
    }
    old_state = sleep;
}

nbfi_status_t NBFI_Init()
{
    
    ax5043_reg_func(AXRADIO_STATUSCHANGE, (void*)axradio_statuschange);

    NBFi_Config_Set_Default();
    for(uint8_t i = 0; i < NBFI_TX_PKTBUF_SIZE; i++) nbfi_TX_pktBuf[i] = 0;
    for(uint8_t i = 0; i < NBFI_RX_PKTBUF_SIZE; i++) nbfi_RX_pktBuf[i] = 0;


    info_timer = dev_info.send_info_interval - 300 - rand()%600;

    RF_Init(nbfi.rx_phy_channel, (rf_antenna_t)nbfi.rx_antenna, nbfi.tx_pwr, nbfi.dl_freq_base);

    if(nbfi.additional_flags&NBFI_OFF_MODE_ON_INIT)
    {
      NBFi_Go_To_Sleep(1);
      ScheduleTask(&nbfi_heartbeat_desc, NBFi_SendHeartBeats, RELATIVE, SECONDS(60));
    }
    else
    {
      NBFi_RX_Controller();
      
      if(!(nbfi.additional_flags&NBFI_FLG_DO_NOT_SEND_PKTS_ON_START))
      { 
            NBFi_Config_Send_Mode(0, NBFI_PARAM_MODE);
            NBFi_Send_Clear_Cmd(0);
      }
      
      NBFi_Force_process();
      __nbfi_measure_voltage_or_temperature(1);
      ScheduleTask(&nbfi_heartbeat_desc, NBFi_SendHeartBeats, RELATIVE, SECONDS(1));
    }
    
    return OK;
}

uint8_t NBFi_can_sleep()
{
  return (!rf_busy) && (rf_state == STATE_OFF) && (NBFi_Packets_To_Send() == 0);
}

#ifdef FORMAT_CODE
#pragma default_function_attributes =
#endif
