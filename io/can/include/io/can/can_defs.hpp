#pragma once

#if __has_include(<linux/can.h>)
#include <linux/can.h>
#else
#include <cstdint>

struct can_frame {
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t __pad;
  uint8_t __res0;
  uint8_t __res1;
  uint8_t data[8];
};

#ifndef CAN_MTU
#define CAN_MTU (sizeof(struct can_frame))
#endif
#ifndef CAN_MAX_DLEN
#define CAN_MAX_DLEN 8
#endif
#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000U
#endif
#ifndef CAN_RTR_FLAG
#define CAN_RTR_FLAG 0x40000000U
#endif
#ifndef CAN_ERR_FLAG
#define CAN_ERR_FLAG 0x20000000U
#endif
#ifndef CAN_SFF_MASK
#define CAN_SFF_MASK 0x000007FFU
#endif
#endif

