//
// Created by tim67 on 2025/11/5.
//

#ifndef FINEMOTE_CAN_HEADER_HPP
#define FINEMOTE_CAN_HEADER_HPP
//以下四个常量复制自HAL库，之后还需要考虑重复定义问题
#include <cstdint>
#define CAN_ID_STD                  (0x00000000U)  /*!< Standard Id */
#define CAN_ID_EXT                  (0x00000004U)  /*!< Extended Id */
#define CAN_RTR_DATA                (0x00000000U)  /*!< Data frame   */
#define CAN_RTR_REMOTE              (0x00000002U)  /*!< Remote frame */

typedef struct{
  uint32_t ID;//CAN与FDCAN的ID没有区别
  uint32_t IDE;//使用CAN_identifier_type
  uint32_t RTR;//使用CAN_remote_transmission_request
  uint32_t DLC;//使用0~8的整数
} FineMote_CAN_HeaderTypeDef;//兼容CAN与FDCAN
#endif // FINEMOTE_CAN_HEADER_HPP
