#include "libmfcrc.h"

#if CRCMODE == 0
#define crc_msb_byte     crc_crc32_msb_byte
#define crc_msbtable     crc_crc32_msbtable
#define crc_msbtable_asm _crc_crc32_msbtable
#else
#error "invalid CRCMODE"
#endif

uint32_t crc_msb_byte(uint32_t crc, uint8_t c)
{
        return (crc << 8) ^ crc_msbtable[((uint8_t)(crc >> 24) ^ c) & (uint8_t)0xff];
}

