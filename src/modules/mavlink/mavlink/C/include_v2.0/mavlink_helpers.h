#ifndef MAVLINK_HELPERS_H_
#define MAVLINK_HELPERS_H_
#include "mavlink_types.h"
#include "checksum.h"
static inline void mavlink_finalize_message(mavlink_message_t* msg, uint8_t sysid, uint8_t compid, uint8_t payload_len, uint8_t incompat_flags, uint8_t compat_flags) {
    msg->magic = 253;
    msg->sysid = sysid;
    msg->compid = compid;
    msg->len = payload_len;
    msg->incompat_flags = incompat_flags;
    msg->compat_flags = compat_flags;
}
#endif
