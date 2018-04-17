#include "libmftypes.h"

int32_t signextend20(int32_t x)
{
	x &= 0xFFFFF;
	x |= -(x & 0x80000);
	return x;
}
