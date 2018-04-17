#include "libmftypes.h"

void fmemset(void *p, char c, uint16_t n)
{
	char *pp = (char *)p;
	for (; n; --n)
		*pp++ = c;
}

