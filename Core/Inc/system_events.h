#ifndef SYSTEM_EVENTS_H
#define SYSTEM_EVENTS_H

#include "cmsis_os2.h"

// 系统事件标志位定义
extern osEventFlagsId_t system_events_handle;

// 事件标志位定义
constexpr uint32_t FLAG_IMU_READY      = 1u << 0;  // IMU数据就绪
constexpr uint32_t FLAG_MOTOR_CTRL     = 1u << 1;  // 电机控制就绪
constexpr uint32_t FLAG_SYSTEM_OK      = 1u << 2;  // 系统状态正常
constexpr uint32_t FLAG_CAN_RX         = 1u << 3;  // CAN数据接收
constexpr uint32_t FLAG_RC_READY       = 1u << 4;  // 遥控器数据就绪
constexpr uint32_t FLAG_EMERGENCY_STOP = 1u << 5;  // 紧急停止

// 系统状态枚举
typedef enum {
    SYSTEM_INIT = 0,
    SYSTEM_READY,
    SYSTEM_RUNNING,
    SYSTEM_ERROR,
    SYSTEM_EMERGENCY_STOP
} SystemState_t;

// 系统状态获取和设置
extern SystemState_t get_system_state();
extern void set_system_state(SystemState_t state);

#endif // SYSTEM_EVENTS_H