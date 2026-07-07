#ifndef MAVLINK_TYPES_H_
#define MAVLINK_TYPES_H_
#include <stdint.h>
typedef struct {
    uint16_t checksum;
    uint8_t magic;
    uint8_t len;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint8_t msgid;
    uint8_t payload[255];
    uint8_t incompat_flags;
    uint8_t compat_flags;
} mavlink_message_t;
typedef struct {
    uint8_t msg_received;
    uint8_t buffer_overrun;
    uint8_t parse_error;
    uint8_t packet_idx;
    uint8_t current_seq;
    uint8_t packet_start;
    uint16_t packet_msgs_received;
} mavlink_status_t;
#endif
