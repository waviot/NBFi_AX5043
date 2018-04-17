#include "libmftypes.h"
#include <stdlib.h>

uint8_t checksignedlimit32(int32_t x, int32_t lim)
{
	if (x < 0) {
		x += lim;
		return x >= 0;
	}
	x -= lim;
	return x <= 0;
}
