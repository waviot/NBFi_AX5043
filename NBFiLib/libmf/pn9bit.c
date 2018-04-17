#include "libmfcrc.h"

uint16_t pn9_advance_bit(uint16_t pn9)
{
	return (uint8_t)(pn9 >> 1) | (((pn9 << 3) ^ (pn9 << 8)) & 0x100);
}

