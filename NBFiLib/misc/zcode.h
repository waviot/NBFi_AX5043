#ifndef ZCODE_H
#define ZCODE_H

#include <libmftypes.h>

#define ZCODE_LEN 16

void ZCODE_Append(uint8_t * src_buf, uint8_t * dst_buf, _Bool parity);

#endif // ZCODE_H
