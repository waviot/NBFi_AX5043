#include "libmfcrc.h"

#if CRCMODE == 0
#define crc_byte      crc_crc8ccitt_byte
#define crc_table     crc_crc8ccitt_table
#define crc_table_asm _crc_crc8ccitt_table
#elif CRCMODE == 1
#define crc_byte      crc_crc8onewire_byte
#define crc_table     crc_crc8onewire_table
#define crc_table_asm _crc_crc8onewire_table
#else
#error "invalid CRCMODE"
#endif

uint8_t crc_byte(uint8_t crc, uint8_t c)
{
        return crc_table[crc ^ c];
}
