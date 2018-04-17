#include "libmftypes.h"
#include <stdlib.h>

uint8_t checksignedlimit16(int16_t x, int16_t lim)
{
	if (x < 0) {
		x += lim;
		return x >= 0;
	}
	x -= lim;
	return x <= 0;
}
