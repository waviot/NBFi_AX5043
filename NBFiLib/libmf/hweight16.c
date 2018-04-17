#include "libmftypes.h"

uint8_t hweight16(uint16_t x)
{
	return hweight8(x) + hweight8(x >> 8);
}
