#include "libmftypes.h"

uint8_t gray_decode8(uint8_t x)
{
	x ^= (x >> 1) & 0x40;
	x ^= (x >> 1) & 0x20;
	x ^= (x >> 1) & 0x10;
	x ^= (x >> 1) & 0x08;
	x ^= (x >> 1) & 0x04;
	x ^= (x >> 1) & 0x02;
	x ^= (x >> 1) & 0x01;
	return x;
}
