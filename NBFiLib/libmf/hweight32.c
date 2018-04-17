#include "libmftypes.h"

uint8_t hweight32(uint32_t x)
{
	return hweight8(x) + hweight8(x >> 8) + hweight8(x >> 16) + hweight8(x >> 24);
}
