#ifndef MAVLINK_CHECKSUM_H_
#define MAVLINK_CHECKSUM_H_
#include <stdint.h>
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum) {
    uint8_t tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}
static inline void crc_init(uint16_t *crcAccum) { *crcAccum = 0xffff; }
static inline uint16_t crc_finalize(uint16_t *crcAccum) { return *crcAccum; }
#endif
