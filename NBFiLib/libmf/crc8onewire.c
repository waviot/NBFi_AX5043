#include "libmfcrc.h"

static uint8_t crc8_reduce(uint8_t idx)
{
	uint8_t cnt = 8;
	do {
		uint8_t m = idx & 0x80;
		idx <<= 1;
		if (m)
			idx ^= 0x31;
	} while (--cnt);
	return idx;
}

uint8_t crc8_onewire_byte(uint8_t crc, uint8_t c)
{
	return crc8_reduce(crc ^ c);
}

uint8_t crc8_onewire(const uint8_t *buf, uint8_t len, uint8_t init)
{
	do {
		init = crc8_reduce(init ^ *buf++);
	} while (len--);
	return init;
}
