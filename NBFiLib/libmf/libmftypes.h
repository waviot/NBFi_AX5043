#ifndef LIBMFTYPES_H
#define LIBMFTYPES_H

#include <stdint.h>

#define LIBMFVERSION 20160321L

/*
 * Delay
 */
extern void delay(uint16_t us);

/*
 * Random Number Generator
 */
//extern uint16_t random_seed;
//extern uint16_t random(void);


extern int32_t signextend12(int16_t x);
extern int32_t signextend16(int16_t x);
extern int32_t signextend20(int32_t x);
extern int32_t signextend24(int32_t x);
extern uint8_t hweight8(uint8_t x);
extern uint8_t hweight16(uint16_t x);
extern uint8_t hweight32(uint32_t x);
#define parity8(x) (hweight8(x) & 1)
#define parity16(x) (hweight16(x) & 1)
#define parity32(x) (hweight32(x) & 1)

extern int16_t signedlimit16(int16_t x, int16_t lim);
extern uint8_t checksignedlimit16(int16_t x, int16_t lim);
extern int32_t signedlimit32(int32_t x, int32_t lim);
extern uint8_t checksignedlimit32(int32_t x, int32_t lim);
extern uint8_t gray_encode8(uint8_t x);
extern uint8_t gray_decode8(uint8_t x);

/*
 * Reverse Bits
 */
extern uint8_t rev8(uint8_t x);

/*
 * fast memset and memcpy
 */

void fmemset(void *p, char c, uint16_t n);
void fmemcpy(void *d, const void *s, uint16_t n);

/*
 * Power Management
 */
 
extern void wtimer_standby(void);
extern void enter_standby(void);
extern void enter_sleep(void);
extern void enter_deepsleep(void);
extern void reset_cpu(void);
extern void turn_off_xosc(void);
extern void turn_off_lpxosc(void);


/*
 * wrnum<xx> Flags
 */
#define WRNUM_SIGNED   0x01
#define WRNUM_PLUS     0x02
#define WRNUM_ZEROPLUS 0x04
#define WRNUM_PADZERO  0x08
#define WRNUM_TSDSEP   0x10
#define WRNUM_LCHEX    0x20

#endif /* LIBMFTYPES_H */
