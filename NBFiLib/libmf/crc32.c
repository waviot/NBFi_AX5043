#include "libmfcrc.h"

#if CRCMODE == 0
#define crc_byte      crc_crc32_byte
#define crc_table     crc_crc32_table
#define crc_table_asm _crc_crc32_table
#else
#error "invalid CRCMODE"
#endif

uint32_t crc_byte(uint32_t crc, uint8_t c)
{
        return (crc >> 8) ^ crc_table[((uint8_t)crc ^ c) & (uint8_t)0xff];
}

