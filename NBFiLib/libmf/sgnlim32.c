#include "libmftypes.h"
#include <stdlib.h>

int32_t signedlimit32(int32_t x, int32_t lim)
{
	if (x < 0) {
		int32_t xx = x + lim;
		if (xx >= 0)
			return x;
		return -lim;
	}
	{
		int32_t xx = x - lim;
		if (xx <= 0)
			return x;
		return lim;
	}
}
