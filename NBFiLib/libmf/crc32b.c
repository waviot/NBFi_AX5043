#include "libmfcrc.h"

#if CRCMODE == 0
#define crc_buf       crc_crc32
#define crc_byte      crc_crc32_byte
#define crc_table_asm _crc_crc32_table
#define CRCMSB        0
#elif CRCMODE == 1
#define crc_buf       crc_crc32_msb
#define crc_byte      crc_crc32_msb_byte
#define crc_table_asm _crc_crc32_msbtable
#define CRCMSB        1
#else
#error "invalid CRCMODE"
#endif

uint32_t crc_buf(const uint8_t *buf, uint16_t buflen, uint32_t crc)
{
	if (!buflen)
		return crc;
	do {
		crc = crc_byte(crc, *buf++);
	} while (--buflen);
	return crc;
}
