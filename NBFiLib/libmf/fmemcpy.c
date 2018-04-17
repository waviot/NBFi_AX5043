#include "libmftypes.h"

void fmemcpy(void *d, const void *s, uint16_t n)
{
	char *pd = (char *)d;
	const char *ps = (const char *)s;
	for (; n; --n)
		*pd++ = *ps++;
}

