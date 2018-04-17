#include "libmftypes.h"

int32_t signextend24(int32_t x)
{
	x &= 0xFFFFFF;
	x |= -(x & 0x800000);
	return x;
}
