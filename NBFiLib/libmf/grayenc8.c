#include "libmftypes.h"

uint8_t gray_encode8(uint8_t x)
{
	x ^= x >> 1;
	return x;
}
