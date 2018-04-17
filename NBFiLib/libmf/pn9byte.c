#include "libmfcrc.h"

uint16_t pn9_advance_byte(uint16_t pn9)
{
	uint8_t bits = 8;
	do {
		pn9 = (uint8_t)(pn9 >> 1) | (((pn9 << 3) ^ (pn9 << 8)) & 0x100);
	} while (--bits);
	return pn9;
}

