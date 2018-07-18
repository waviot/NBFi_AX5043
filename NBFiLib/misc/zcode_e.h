#ifndef ZCODE_E_H
#define ZCODE_E_H

#include <libmftypes.h>

#define ZCODE_E_LEN 32

void ZCODE_E_Append(uint8_t * src_buf, uint8_t * dst_buf, _Bool parity);

#endif // ZCODE_E_H
