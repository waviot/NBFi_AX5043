#include "libmfcrc.h"

#if CRCMODE == 0
#define crc_buf       crc_crc16
#define crc_byte      crc_crc16_byte
#define crc_table_asm _crc_crc16_table
#define CRCMSB        0
#elif CRCMODE == 1
#define crc_buf       crc_crc16_msb
#define crc_byte      crc_crc16_msb_byte
#define crc_table_asm _crc_crc16_msbtable
#define CRCMSB        1
#elif CRCMODE == 2
#define crc_buf       crc_crc16dnp
#define crc_byte      crc_crc16dnp_byte
#define crc_table_asm _crc_crc16dnp_table
#define CRCMSB        0
#elif CRCMODE == 3
#define crc_buf       crc_crc16dnp_msb
#define crc_byte      crc_crc16dnp_msb_byte
#define crc_table_asm _crc_crc16dnp_msbtable
#define CRCMSB        1
#elif CRCMODE == 4
#define crc_buf       crc_ccitt
#define crc_byte      crc_ccitt_byte
#define crc_table_asm _crc_ccitt_table
#define CRCMSB        0
#elif CRCMODE == 5
#define crc_buf       crc_ccitt_msb
#define crc_byte      crc_ccitt_msb_byte
#define crc_table_asm _crc_ccitt_msbtable
#define CRCMSB        1
#else
#error "invalid CRCMODE"
#endif

uint16_t crc_buf(const uint8_t *buf, uint16_t buflen, uint16_t crc)
{
	if (!buflen)
		return crc;
	do {
		crc = crc_byte(crc, *buf++);
	} while (--buflen);
	return crc;
}
