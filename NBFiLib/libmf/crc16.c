#include "libmfcrc.h"

#if CRCMODE == 0
#define crc_byte      crc_crc16_byte
#define crc_table     crc_crc16_table
#define crc_table_asm _crc_crc16_table
#elif CRCMODE == 1
#define crc_byte      crc_crc16dnp_byte
#define crc_table     crc_crc16dnp_table
#define crc_table_asm _crc_crc16dnp_table
#elif CRCMODE == 2
#define crc_byte      crc_ccitt_byte
#define crc_table     crc_ccitt_table
#define crc_table_asm _crc_ccitt_table
#else
#error "invalid CRCMODE"
#endif

uint16_t crc_byte(uint16_t crc, uint8_t c)
{
        return (crc >> 8) ^ crc_table[((uint8_t)crc ^ c) & (uint8_t)0xff];
}
