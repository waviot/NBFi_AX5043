#include "libmftypes.h"

int32_t signextend12(int16_t x)
{
	x &= 0xFFF;
	x |= -(x & 0x800);
	return x;
}
