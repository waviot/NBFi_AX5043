#ifndef LIBMFCRC_H
#define LIBMFCRC_H

#include "libmftypes.h"

/*
 * CRC-16 CCITT
 */
extern uint16_t crc_ccitt_byte(uint16_t crc, uint8_t c);
extern uint16_t crc_ccitt_msb_byte(uint16_t crc, uint8_t c);
extern uint16_t crc_ccitt(const uint8_t *buf, uint16_t buflen, uint16_t crc);
extern uint16_t crc_ccitt_msb(const uint8_t *buf, uint16_t buflen, uint16_t crc);
extern const uint16_t crc_ccitt_table[256];
extern const uint16_t crc_ccitt_msbtable[256];

/*
 * CRC-16
 */
extern uint16_t crc_crc16_byte(uint16_t crc, uint8_t c);
extern uint16_t crc_crc16_msb_byte(uint16_t crc, uint8_t c);
extern uint16_t crc_crc16(const uint8_t *buf, uint16_t buflen, uint16_t crc);
extern uint16_t crc_crc16_msb(const uint8_t *buf, uint16_t buflen, uint16_t crc);
extern const uint16_t crc_crc16_table[256];
extern const uint16_t crc_crc16_msbtable[256];

/*
 * CRC-16 DNP
 */
extern uint16_t crc_crc16dnp_byte(uint16_t crc, uint8_t c);
extern uint16_t crc_crc16dnp_msb_byte(uint16_t crc, uint8_t c);
extern uint16_t crc_crc16dnp(const uint8_t *buf, uint16_t buflen, uint16_t crc);
extern uint16_t crc_crc16dnp_msb(const uint8_t *buf, uint16_t buflen, uint16_t crc);
extern const uint16_t crc_crc16dnp_table[256];
extern const uint16_t crc_crc16dnp_msbtable[256];

/*
 * CRC-32
 */
extern uint32_t crc_crc32_byte(uint32_t crc, uint8_t c);
extern uint32_t crc_crc32_msb_byte(uint32_t crc, uint8_t c);
extern uint32_t crc_crc32(const uint8_t *buf, uint16_t buflen, uint32_t crc);
extern uint32_t crc_crc32_msb(const uint8_t *buf, uint16_t buflen, uint32_t crc);
extern const uint32_t crc_crc32_table[256];
extern const uint32_t crc_crc32_msbtable[256];

/*
 * CRC-8 CCITT
 */

extern uint8_t crc_crc8ccitt_byte(uint8_t crc, uint8_t c);
extern uint8_t crc_crc8ccitt_msb_byte(uint8_t crc, uint8_t c);
extern uint8_t crc_crc8ccitt(const uint8_t *buf, uint16_t buflen, uint8_t crc);
extern uint8_t crc_crc8ccitt_msb(const uint8_t *buf, uint16_t buflen, uint8_t crc);
extern const uint8_t crc_crc8ccitt_table[256];
extern const uint8_t crc_crc8ccitt_msbtable[256];

/*
 * CRC-8 OneWire
 */

extern uint8_t crc_crc8onewire_byte(uint8_t crc, uint8_t c);
extern uint8_t crc_crc8onewire_msb_byte(uint8_t crc, uint8_t c);
extern uint8_t crc_crc8onewire(const uint8_t *buf, uint16_t buflen, uint8_t crc);
extern uint8_t crc_crc8onewire_msb(const uint8_t *buf, uint16_t buflen, uint8_t crc);
extern const uint8_t crc_crc8onewire_table[256];
extern const uint8_t crc_crc8onewire_msbtable[256];

/*
 * CRC-8 CCITT, non-table driven routines (slower, but more compact)
 */
extern uint8_t crc8_ccitt_byte(uint8_t crc, uint8_t c);
extern uint8_t crc8_ccitt(const uint8_t *buf, uint8_t len, uint8_t init);

/*
 * CRC-8 OneWire, non-table driven routines (slower, but more compact)
 */
extern uint8_t crc8_onewire_byte(uint8_t crc, uint8_t c);
extern uint8_t crc8_onewire(const uint8_t *buf, uint8_t len, uint8_t init);

/*
 * PN9 Whitening Sequence
 */
extern uint16_t pn9_advance(uint16_t pn9);
extern const uint8_t pn9_table[512];
extern uint16_t pn9_advance_bit(uint16_t pn9);
extern uint16_t pn9_advance_bits(uint16_t pn9, uint16_t bits);
extern uint16_t pn9_advance_byte(uint16_t pn9);
extern uint16_t pn9_buffer(uint8_t *buf, uint16_t buflen, uint16_t pn9, uint8_t xor);

/*
 * PN15 Whitening Sequence
 */
extern uint16_t pn15_advance(uint16_t pn15);
extern uint8_t pn15_output(uint16_t pn15);
extern const uint16_t pn15_adv_table[256];
extern const uint8_t pn15_out_table[256];

#endif /* LIBMFCRC_H */
