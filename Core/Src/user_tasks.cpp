//
// #include "user_tasks.h"
// #include "cmsis_os2.h"
// #include "string.h"
// #include "can.h"
// #include "stm32f4xx_hal_can.h"
// // #include "iwdg.h"
// #include "imu.h"
// #include "rc.h"
//
//
// osEventFlagsAttr_t test_event_flags_attributes = {.name = "test_event_flags"};
// osEventFlagsId_t test_event_flags_handle;
//
// constexpr float dt = 0.001f;
// constexpr float kg = 0.1f;
// constexpr float g_threshold = 0.1f;
// constexpr float gyro_bias[3] = { 0.0f, 0.0f, 0.0f };
// constexpr float r_imu[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
//
//
//
// //Gimbal gimbal_controller;
// IMU imu_sensor(dt, kg, g_threshold, r_imu, gyro_bias);
// //RemoteControl rc_controller;
//
// uint8_t rx_buf[18];
// uint8_t rx_data[18];
//
//
//
//
// // Control任务 - 主控制循环
// osThreadId_t control_task_handle;
// constexpr osThreadAttr_t control_task_attributes = {
//     .name = "control_task",
//     .stack_size = 256 * 4,  // 1KB栈空间
//     .priority = osPriorityHigh,  // 较高优先级
// };
//
// // CAN任务 - 数据收发
// osThreadId_t can_send_task_handle;
// constexpr osThreadAttr_t can_send_task_attributes = {
//     .name = "can_task",
//     .stack_size = 192 * 4,  // 768字节栈空间
//     .priority = osPriorityAboveNormal,  // 较高优先级保证通信实时性
// };
//
// osThreadId_t can_recv_task_handle;
// constexpr osThreadAttr_t can_recv_task_attributes = {
//     .name = "can_task",
//     .stack_size = 192 * 4,  // 768字节栈空间
//     .priority = osPriorityAboveNormal,  // 较高优先级保证通信实时性
// };
//
// // IMU任务 - 姿态解算
// osThreadId_t imu_task_handle;
// constexpr osThreadAttr_t imu_task_attributes = {
//     .name = "imu_task",
//     .stack_size = 320 * 4,  // 1.28KB栈空间（姿态解算需要较多栈空间）
//     .priority = osPriorityNormal,
// };
//
// // Motor任务 - 电机控制
// osThreadId_t motor_task_handle;
// constexpr osThreadAttr_t motor_task_attributes = {
//     .name = "motor_task",
//     .stack_size = 192 * 4,  // 768字节栈空间
//     .priority = osPriorityRealtime,  // 最高优先级保证电机控制实时性
// };
//
// // Control任务函数 - 主控制循环
// [[noreturn]] void control_task(void *) {
//     // 初始化代码
//     //system_ticks = 0;
//     //uint32_t ticks = osKernelGetTickCount();
//
//     while (true) {
//         const auto tick = osKernelGetTickCount();
//
//         // 1. 等待IMU数据就绪
//         osEventFlagsWait(system_events_handle, flag_imu_ready, osFlagsWaitAny, osWaitForever);
//
//         // 2. 执行控制算法
//         control_algorithm();
//
//         // 3. 设置电机控制标志
//         osEventFlagsSet(system_events_handle, flag_motor_ctrl);
//
//         // 4. 系统状态监测
//         system_monitoring();
//
//         // 5. 设置系统正常标志
//         osEventFlagsSet(system_events_handle, flag_system_ok);
//
//         // 固定频率控制（例如100Hz）
//         osDelayUntil(tick + 10);  // 10ms周期
//     }
// }
//
// // CAN任务函数 - 数据通信
// [[noreturn]] void can_send_task(void *) {
//     // CAN初始化代码
//
//     uint32_t tick = osKernelGetTickCount();
//     CAN_TxHeaderTypeDef tx_header;
//     uint8_t tx_data[8];
//     uint32_t tx_mailbox;
//
//
//     // can_init();
//     //
//     // while (true) {
//     //     const auto tick = osKernelGetTickCount();
//     //
//     //     // 1. CAN数据发送
//     //     can_send_motor_command(motor_speed);
//     //     can_send_system_status();
//     //
//     //     // 2. CAN数据接收处理
//     //     if (can_receive_data(can_rx_data)) {
//     //         // 设置CAN接收标志
//     //         osEventFlagsSet(system_events_handle, flag_can_rx);
//     //         can_data_processing(can_rx_data);
//     //     }
//     //
//     //     // 3. 通信状态监测
//     //     can_communication_check();
//
//         // 固定频率运行（例如50Hz）
//         osDelayUntil(tick + 20);  // 20ms周期
//     }
// }
//
//
// [[noreturn]] void can_recv_task(void *) {
//
// }
//
//
//
// // IMU任务函数 - 姿态解算
// [[noreturn]] void imu_task(void *) {
//     // IMU传感器初始化
//     uint32_t tick = osKernelGetTickCount();
//     for (;;)
//     {
//         imu_sensor.ReadSensor();
//
//         imu_sensor.UpdateAttitude();
//
//
//
//         osDelayUntil(tick += 1);
//     }
//
//         // 2. 姿态
//     }
//
//
//
//
// [[noreturn]] void motor_task(void *) {
//
//     }
//
//
//
//
//
//
//
//     void user_tasks_init() {
//         // test_task_handle = osThreadNew(test_task, nullptr, &test_task_attributes); // 创建任务
//         // test_semaphore_handle = osSemaphoreNew(1, 0, &test_semaphore_attributes);
//         // test_event_flags_handle = osEventFlagsNew(&test_event_flags_attributes);
//         control_task_handle = osThreadNew(control_task, nullptr, &control_task_attributes);
//         can_send_task_handle = osThreadNew(can_send_task, nullptr, &can_send_task_attributes);
//         can_recv_task_handle = osThreadNew(can_recv_task, nullptr, &can_recv_task_attributes);
//         imu_task_handle = osThreadNew(imu_task, nullptr, &imu_task_attributes);
//         motor_task_handle = osThreadNew(motor_task, nullptr, &motor_task_attributes);
//         // test2_task_handle = osThreadNew(test2_task, nullptr, &test2_task_attributes);
//     }



// C++
#include "user_tasks.h"
#include "cmsis_os2.h"
#include "string.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "imu.h"
#include "rc.h"
#include "motor.h"
#include "FreeRTOS.h"
// #include "gimbal_controller.h"
#include "CanTxManager.h"



IMU imu; // 创建 IMU 实例
// gimbal_controller.set_imu(&imu_instance);
//================= 系统事件与控制算法占位 =================

extern CAN_TxHeaderTypeDef tx_header_1;
extern uint8_t tx_data_1[8];
extern Motor Motor_pitch;
extern Motor Motor_yaw;



uint8_t tick1 = 0;
uint8_t tick2 = 0;
uint8_t tick3 = 0;
uint8_t tick4 = 0;
uint8_t tick5 = 0;
uint8_t tick6 = 0;

// 事件标志位（将来可移到独立的 system_events.h / .cpp）
osEventFlagsAttr_t system_events_attr = { .name = "system_events" };
osEventFlagsId_t system_events_handle = nullptr;

constexpr uint32_t flag_imu_ready   = 1u << 0;
constexpr uint32_t flag_motor_ctrl  = 1u << 1;
constexpr uint32_t flag_system_ok   = 1u << 2;
// 预留：比如 CAN 接收完成标志
// constexpr uint32_t flag_can_rx      = 1u << 3;

// 控制算法占位实现（将来可移动到 algorithm 模块）
void control_algorithm() {
    // TODO: 在此调用姿态、遥控、电机等模块的控制逻辑
}

// 系统监控占位实现（将来可移动到独立模块）
void system_monitoring() {
    // TODO: 看门狗、故障检测、温度、电压等监控逻辑
}

//================= IMU 实例与缓冲区 =================

constexpr float dt = 0.001f;
constexpr float kg = 0.1f;
constexpr float g_threshold = 0.1f;
constexpr float gyro_bias[3] = { 0.0f, 0.0f, 0.0f };
constexpr float r_imu[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

IMU imu_sensor(dt, kg, g_threshold, r_imu, gyro_bias);
uint8_t rx_buf[18];
// uint8_t rx_data[18];

//================= 任务属性定义 =================

// Control任务 - 主控制循环
osThreadId_t control_task_handle;
constexpr osThreadAttr_t control_task_attributes = {
    .name       = "control_task",
    .stack_size = 256 * 4,
    .priority   = osPriorityHigh,
};

// CAN发送任务
osThreadId_t can_send_task_handle;
constexpr osThreadAttr_t can_send_task_attributes = {
    .name       = "can_task_send",
    .stack_size = 192 * 4,
    .priority   = osPriorityAboveNormal,
};

// CAN接收任务
osThreadId_t can_recv_task_handle;
constexpr osThreadAttr_t can_recv_task_attributes = {
    .name       = "can_task_recv",
    .stack_size = 192 * 4,
    .priority   = osPriorityAboveNormal,
};

// IMU任务 - 姿态解算
osThreadId_t imu_task_handle;
constexpr osThreadAttr_t imu_task_attributes = {
    .name       = "imu_task",
    .stack_size = 320 * 4,
    .priority   = osPriorityNormal,
};

// Motor任务 - 电机控制
osThreadId_t motor_task_handle;
constexpr osThreadAttr_t motor_task_attributes = {
    .name       = "motor_task",
    .stack_size = 192 * 4,
    .priority   = osPriorityRealtime,
};

//遥控器任务 - 遥控器数据处理
osThreadId_t rc_task_handle;
constexpr osThreadAttr_t rc_task_attributes = {
    .name       = "rc_task",
    .stack_size = 192 * 4,
    .priority   = osPriorityNormal,
};

//================= 任务函数实现 =================

// Control任务函数 - 主控制循环
[[noreturn]] void control_task(void *) {
    const uint32_t period_ms = 10; // 100 Hz
    uint32_t tick = osKernelGetTickCount();

    for (;;) {
        tick1++;
        // 1. 等待 IMU 数据就绪
        osEventFlagsWait(system_events_handle, flag_imu_ready,
                         osFlagsWaitAny, osWaitForever);

        // 2. 执行控制算法
        control_algorithm();

        // 3. 通知电机控制任务可以更新电机输出
        osEventFlagsSet(system_events_handle, flag_motor_ctrl);

        // 4. 系统状态监测
        system_monitoring();

        // 5. 标记系统状态正常
        osEventFlagsSet(system_events_handle, flag_system_ok);

        osDelayUntil(tick += period_ms);
    }
}


uint32_t tick;
// CAN发送任务 - 数据通信
[[noreturn]] void can_send_task(void *) {
    const uint32_t period_ms = 20; // 50 Hz
    tick = osKernelGetTickCount();


    // uint8_t tx_data[8]{};
    // uint32_t tx_mailbox = 0;

    for (;;) {
        tick2++;
        // TODO: 根据控制结果封装并发送 CAN 数据
        HAL_CAN_AddTxMessage(&hcan1, &tx_header_1, tx_data_1, &can_tx_mail_box_);


        osDelayUntil(tick += period_ms);
    }
}

// CAN接收任务
[[noreturn]] void can_recv_task(void *) {
    const uint32_t period_ms = 10;
    uint32_t tick = osKernelGetTickCount();

    for (;;) {
        tick3++;
        // TODO: 轮询/中断方式接收 CAN，解析后存入 rc / motor / status 等模块
        // if (can_receive_data(rx_data)) {
        //     osEventFlagsSet(system_events_handle, flag_can_rx);
        // }

        osDelayUntil(tick += period_ms);
    }
}

// IMU任务函数 - 姿态解算
[[noreturn]] void imu_task(void *) {
    const uint32_t period_ms = 1; // 1 kHz，根据硬件情况调整
    uint32_t tick = osKernelGetTickCount();

    // 根据需要设置初始欧拉角（单位：度），这里全部置 0
    EulerAngle_t init_angle(0.0f, 0.0f, 0.0f);
    imu_sensor.init(init_angle);

    for (;;) {
        tick4++;
        // 读取 BMI088 传感器数据
        imu_sensor.readSensor();
  
        // 使用 Mahony 算法更新姿态
        // imu_sensor.update();

        // 通知控制任务 IMU 数据已更新
        osEventFlagsSet(system_events_handle, flag_imu_ready);

        osDelayUntil(tick += period_ms);
    }
}

// Motor任务函数 - 电机控制
[[noreturn]] void motor_task(void *) {
    const uint32_t period_ms = 5;
    uint32_t tick = osKernelGetTickCount();

    for (;;) {
        tick5++;
        // 如果需要基于事件触发，也可以在这里等待 flag_motor_ctrl
        // osEventFlagsWait(system_events_handle, flag_motor_ctrl, osFlagsWaitAny, osWaitForever);
        //
        // TODO: 读取控制量，更新电机（通过 CAN / PWM 等）
        // motor_update();
        Motor_yaw.handle();
        osDelayUntil(tick += period_ms);
    }
}

[[noreturn]] void rc_task(void *) {
    const uint32_t period_ms = 10;
    uint32_t tick = osKernelGetTickCount();

    for (;;) {
        tick6++;
        // 1. 读取遥控器数据
        // rc.handle(rx_data);

        // 2. 获取控制量
        // float pitch = rc.get_pitch_input();
        // float yaw = rc.get_yaw_input();

        // 3. 通知控制任务更新控制量
    }
}
//================= 任务初始化 =================

void user_tasks_init() {
    // 创建系统事件标志组
    system_events_handle = osEventFlagsNew(&system_events_attr);

    control_task_handle   = osThreadNew(control_task,   nullptr, &control_task_attributes);
    can_send_task_handle  = osThreadNew(can_send_task, nullptr, &can_send_task_attributes);
    can_recv_task_handle  = osThreadNew(can_recv_task, nullptr, &can_recv_task_attributes);
    imu_task_handle       = osThreadNew(imu_task,      nullptr, &imu_task_attributes);
    motor_task_handle     = osThreadNew(motor_task,    nullptr, &motor_task_attributes);
    rc_task_handle = osThreadNew(rc_task, nullptr, &rc_task_attributes);
}



































