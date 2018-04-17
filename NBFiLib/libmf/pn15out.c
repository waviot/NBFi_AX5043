#include "libmfcrc.h"

uint8_t pn15_output(uint16_t pn15)
{
	return pn15_out_table[pn15 & 0xff];
}
