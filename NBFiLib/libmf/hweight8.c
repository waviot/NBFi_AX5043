#include "libmftypes.h"

uint8_t hweight8(uint8_t x)
{
	x = (x & 0x55) + ((x >> 1) & 0x55);
	x = (x & 0x33) + ((x >> 2) & 0x33);
	x = (x & 0x0F) + ((x >> 4) & 0x0F);
	return x;
}
	