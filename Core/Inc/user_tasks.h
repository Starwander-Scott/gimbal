//
// Created by cycsjtuer on 2025/11/11.
//




// C++
#pragma once

#include "cmsis_os2.h"

// 供其它模块使用的系统事件与标志位
extern osEventFlagsId_t system_events_handle;
extern const uint32_t flag_imu_ready;
extern const uint32_t flag_motor_ctrl;
extern const uint32_t flag_system_ok;

#ifdef __cplusplus
extern "C" {
#endif
    void user_tasks_init();
#ifdef __cplusplus
}
#endif


// // 任务入口函数（CMSIS-RTOS2 要求 C 接口）
// [[noreturn]] void control_task(void *);
// [[noreturn]] void can_send_task(void *);
// [[noreturn]] void can_recv_task(void *);
// [[noreturn]] void imu_task(void *);
// [[noreturn]] void motor_task(void *);

// 控制算法和系统监控接口
void control_algorithm();
void system_monitoring();




























// #ifndef FINAL_USER_TASKS_H
// #define FINAL_USER_TASKS_H
//
//
// #ifdef __cplusplus
// extern "C" {
// #endif
//
//     void user_tasks_init();
//
// #ifdef __cplusplus
// }
// #endif
//
//
//
//
//
//
// #endif //FINAL_USER_TASKS_H