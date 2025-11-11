
#include "user_tasks.h"
#include "cmsis_os2.h"




// Control任务 - 主控制循环
osThreadId_t control_task_handle;
constexpr osThreadAttr_t control_task_attributes = {
    .name = "control_task",
    .stack_size = 256 * 4,  // 1KB栈空间
    .priority = osPriorityHigh,  // 较高优先级
};

// CAN任务 - 数据收发
osThreadId_t can_task_handle;
constexpr osThreadAttr_t can_task_attributes = {
    .name = "can_task",
    .stack_size = 192 * 4,  // 768字节栈空间
    .priority = osPriorityAboveNormal,  // 较高优先级保证通信实时性
};

// IMU任务 - 姿态解算
osThreadId_t imu_task_handle;
constexpr osThreadAttr_t imu_task_attributes = {
    .name = "imu_task",
    .stack_size = 320 * 4,  // 1.28KB栈空间（姿态解算需要较多栈空间）
    .priority = osPriorityNormal,
};

// Motor任务 - 电机控制
osThreadId_t motor_task_handle;
constexpr osThreadAttr_t motor_task_attributes = {
    .name = "motor_task",
    .stack_size = 192 * 4,  // 768字节栈空间
    .priority = osPriorityRealtime,  // 最高优先级保证电机控制实时性
};

// Control任务函数 - 主控制循环
[[noreturn]] void control_task(void *) {
    // 初始化代码
    system_ticks = 0;

    while (true) {
        const auto tick = osKernelGetTickCount();

        // 1. 等待IMU数据就绪
        osEventFlagsWait(system_events_handle, flag_imu_ready, osFlagsWaitAny, osWaitForever);

        // 2. 执行控制算法
        control_algorithm();

        // 3. 设置电机控制标志
        osEventFlagsSet(system_events_handle, flag_motor_ctrl);

        // 4. 系统状态监测
        system_monitoring();

        // 5. 设置系统正常标志
        osEventFlagsSet(system_events_handle, flag_system_ok);

        // 固定频率控制（例如100Hz）
        osDelayUntil(tick + 10);  // 10ms周期
    }
}

// CAN任务函数 - 数据通信
[[noreturn]] void can_task(void *) {
    // CAN初始化代码
    can_init();

    while (true) {
        const auto tick = osKernelGetTickCount();

        // 1. CAN数据发送
        can_send_motor_command(motor_speed);
        can_send_system_status();

        // 2. CAN数据接收处理
        if (can_receive_data(can_rx_data)) {
            // 设置CAN接收标志
            osEventFlagsSet(system_events_handle, flag_can_rx);
            can_data_processing(can_rx_data);
        }

        // 3. 通信状态监测
        can_communication_check();

        // 固定频率运行（例如50Hz）
        osDelayUntil(tick + 20);  // 20ms周期
    }
}

// IMU任务函数 - 姿态解算
[[noreturn]] void imu_task(void *) {
    // IMU传感器初始化
    imu_sensor_init();

    while (true) {
        const auto tick = osKernelGetTickCount();

        // 1. 读取IMU原始数据
        float gyro[3], accel[3], mag[3];
        imu_read_data(gyro, accel, mag);

        // 2. 姿态
    }
[[noreturn]] void motor_task(void *) {

    }

    void user_tasks_init() {
        // test_task_handle = osThreadNew(test_task, nullptr, &test_task_attributes); // 创建任务
        // test_semaphore_handle = osSemaphoreNew(1, 0, &test_semaphore_attributes);
        // test_event_flags_handle = osEventFlagsNew(&test_event_flags_attributes);
        control_task_handle = osThreadNew(control_task, nullptr, &control_task_attributes);
        can_task_handle = osThreadNew(can_task, nullptr, &can_task_attributes);
        imu_task_handle = osThreadNew(imu_task, nullptr, &imu_task_attributes);
        motor_task_handle = osThreadNew(motor_task, nullptr, &motor_task_attributes);
        // test2_task_handle = osThreadNew(test2_task, nullptr, &test2_task_attributes);
    }









































//
// //
// // Created by cycsjtuer on 2025/11/11.
// //
// //
// // Created by cycsjtuer on 2025/11/1.
// //
//
// #include "user_tasks.h"
// #include "cmsis_os2.h"
//
//
// uint32_t count = 0;
// uint32_t send1 = 0;
// uint32_t send2 = 0;
// uint32_t recv = 0;
// constexpr auto flag_1 = 1u << 0;
// constexpr auto flag_2 = 1u << 1;
//
//
// osSemaphoreAttr_t test_semaphore_attributes = {.name = "test_semaphore"};
// osSemaphoreId_t test_semaphore_handle;
// osEventFlagsAttr_t test_event_flags_attributes = {.name = "test_event_flags"};
// osEventFlagsId_t test_event_flags_handle;
//
//
// osThreadId_t test_task_handle; // 任务句柄，用于引用任务
// constexpr osThreadAttr_t test_task_attributes = {
//     .name = "test_task", // 任务名称，便于调试
//     .stack_size = 128 * 4, // 栈大小=512字节（128字*4字节/字）
//     .priority = osPriorityNormal, // 优先级设为普通
// };
//
//
// osThreadId_t test2_task_handle; // 任务句柄，用于引用任务
// constexpr osThreadAttr_t test2_task_attributes = {
//     .name = "test2_task", // 任务名称，便于调试
//     .stack_size = 128 * 4, // 栈大小=512字节（128字*4字节/字）
//     .priority = osPriorityNormal, // 优先级设为普通
// };
//
// osThreadId_t test3_task_handle; // 任务句柄，用于引用任务
// constexpr osThreadAttr_t test3_task_attributes = {
//     .name = "test3_task", // 任务名称，便于调试
//     .stack_size = 128 * 4, // 栈大小=512字节（128字*4字节/字）
//     .priority = osPriorityNormal, // 优先级设为普通
// };
//
// //
// // [[noreturn]] void test_task(void *) {
// //     while (true) {
// //         const auto tick = osKernelGetTickCount(); // 获取当前系统tick计数
// //         ++count; // 递增全局计数器
// //         osDelayUntil(tick + 1); // 延迟直到下一个tick周期
// //     }
// // }
// //
// //
//
// [[noreturn]] void test_task(void *) {
//     // while (true) {
//     //     const auto tick = osKernelGetTickCount();
//     //     if (send++ % 5 == 0) {
//     //         osSemaphoreRelease(test_semaphore_handle);
//     //     }
//     //     osDelayUntil(tick + 1);
//     // }
//     while (true) {
//         const auto tick = osKernelGetTickCount();
//         send1++;
//         if (send1 % 7 == 0) {
//             osEventFlagsSet(test_event_flags_handle, flag_1);
//         } else if (send1 % 7 == 1) {
//             osEventFlagsClear(test_event_flags_handle, flag_1);
//         }
//         osDelayUntil(tick + 100);
//     }
//     //
//     // osEventFlagsWait(test_event_flags_handle, flag_1 | flag_2,osFlagsWaitAll,osWaitForever);
//     // osEventFlagsClear(test_event_flags_handle, flag_1 | flag_2);
//     // osEventFlagsSet(test_event_flags_handle, flag_1 | flag_2);
// }
//
// [[noreturn]] void test2_task(void *) {
//     while (true) {
//         const auto tick = osKernelGetTickCount();
//         send2++;
//         osEventFlagsSet(test_event_flags_handle, flag_2);
//         osDelayUntil(tick + 100);
//     }
// }
//
// [[noreturn]] void test3_task(void *) {
//     while (true) {
//         osEventFlagsWait(test_event_flags_handle, flag_1 | flag_2,osFlagsWaitAll,osWaitForever);
//         recv++;
//         osEventFlagsClear(test_event_flags_handle, flag_1 | flag_2);
//     }
// }
//
// // [[noreturn]] void test2_task(void *) {
// //     while (true) {
// //         osSemaphoreAcquire(test_semaphore_handle,osWaitForever);
// //         recv++;
// //     }
// // }
//
//
// void user_tasks_init() {
//     // test_task_handle = osThreadNew(test_task, nullptr, &test_task_attributes); // 创建任务
//     // test_semaphore_handle = osSemaphoreNew(1, 0, &test_semaphore_attributes);
//     test_event_flags_handle = osEventFlagsNew(&test_event_flags_attributes);
//     test_task_handle = osThreadNew(test_task, nullptr, &test_task_attributes);
//     test2_task_handle = osThreadNew(test2_task, nullptr, &test2_task_attributes);
//     test3_task_handle = osThreadNew(test3_task, nullptr, &test3_task_attributes);
//     // test2_task_handle = osThreadNew(test2_task, nullptr, &test2_task_attributes);
// }




