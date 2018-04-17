#include "libmfcrc.h"

uint16_t pn9_buffer(uint8_t *buf, uint16_t buflen, uint16_t pn9, uint8_t xor)
{
	if (!buflen)
		return pn9;
	do {
		*buf++ ^= pn9 ^ xor;
		pn9 = pn9_advance_byte(pn9);
	} while (--buflen);
	return pn9;
}

