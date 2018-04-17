#include "libmfcrc.h"

#if CRCMODE == 0
#define crc_msb_byte     crc_crc8ccitt_msb_byte
#define crc_msbtable     crc_crc8ccitt_msbtable
#define crc_msbtable_asm _crc_crc8ccitt_msbtable
#elif CRCMODE == 1
#define crc_msb_byte     crc_crc8onewire_msb_byte
#define crc_msbtable     crc_crc8onewire_msbtable
#define crc_msbtable_asm _crc_crc8onewire_msbtable
#else
#error "invalid CRCMODE"
#endif

uint8_t crc_msb_byte(uint8_t crc, uint8_t c)
{
        return crc_msbtable[crc ^ c];
}

