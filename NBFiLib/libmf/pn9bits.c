#include "libmfcrc.h"

uint16_t pn9_advance_bits(uint16_t pn9, uint16_t bits)
{
	if (!bits)
		return pn9;
	do {
		pn9 = (uint8_t)(pn9 >> 1) | (((pn9 << 3) ^ (pn9 << 8)) & 0x100);
	} while (--bits);
	return pn9;
}

