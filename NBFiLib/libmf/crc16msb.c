#include "libmfcrc.h"

#if CRCMODE == 0
#define crc_msb_byte     crc_crc16_msb_byte
#define crc_msbtable     crc_crc16_msbtable
#define crc_msbtable_asm _crc_crc16_msbtable
#elif CRCMODE == 1
#define crc_msb_byte     crc_crc16dnp_msb_byte
#define crc_msbtable     crc_crc16dnp_msbtable
#define crc_msbtable_asm _crc_crc16dnp_msbtable
#elif CRCMODE == 2
#define crc_msb_byte     crc_ccitt_msb_byte
#define crc_msbtable     crc_ccitt_msbtable
#define crc_msbtable_asm _crc_ccitt_msbtable
#else
#error "invalid CRCMODE"
#endif

uint16_t crc_msb_byte(uint16_t crc, uint8_t c)
{
        return (crc << 8) ^ crc_msbtable[((uint8_t)(crc >> 8) ^ c) & (uint8_t)0xff];
}

