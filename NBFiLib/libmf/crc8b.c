#include "libmfcrc.h"

#if CRCMODE == 0
#define crc_buf       crc_crc8ccitt
#define crc_byte      crc_crc8ccitt_byte
#define crc_table_asm _crc_crc8ccitt_table
#define CRCMSB        0
#elif CRCMODE == 1
#define crc_buf       crc_crc8ccitt_msb
#define crc_byte      crc_crc8ccitt_msb_byte
#define crc_table_asm _crc_crc8ccitt_msbtable
#define CRCMSB        1
#elif CRCMODE == 2
#define crc_buf       crc_crc8onewire
#define crc_byte      crc_crc8onewire_byte
#define crc_table_asm _crc_crc8onewire_table
#define CRCMSB        0
#elif CRCMODE == 3
#define crc_buf       crc_crc8onewire_msb
#define crc_byte      crc_crc8onewire_msb_byte
#define crc_table_asm _crc_crc8onewire_msbtable
#define CRCMSB        1
#else
#error "invalid CRCMODE"
#endif

uint8_t crc_buf(const uint8_t *buf, uint16_t buflen, uint8_t crc)
{
	if (!buflen)
		return crc;
	do {
		crc = crc_byte(crc, *buf++);
	} while (--buflen);
	return crc;
}
