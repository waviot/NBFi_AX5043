#ifndef XTEA_H
#define XTEA_H

#include <libmftypes.h>

void XTEA_Encode(uint8_t* buf);

void XTEA_Decode(uint8_t* buf);

_Bool XTEA_Available();

_Bool XTEA_Enabled();

void XTEA_Enable(_Bool enable);

void XTEA_Set_KEY(uint32_t* key);

void XTEA_Set_KEY_Ptr(uint32_t* ptr);

#endif
