#include "libmftypes.h"
#include <stdlib.h>

int16_t signedlimit16(int16_t x, int16_t lim)
{
	if (x < 0) {
		int16_t xx = x + lim;
		if (xx >= 0)
			return x;
		return -lim;
	}
	{
		int16_t xx = x - lim;
		if (xx <= 0)
			return x;
		return lim;
	}
}

