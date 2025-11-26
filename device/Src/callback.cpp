//
// Created by cycsjtuer on 2025/11/11.
//

#include <stdint.h>
#include "usart.h"
#include "main.h"
#include "rc.h"
#include "can.h"
#include "cmsis_os2.h"
#include "gpio.h"
#include "math.h"
#include "motor.h"
#include "tim.h"
#include "imu.h"
// #include "interrupt_notify.h"


// void HAL_GPIO_EXTI_Callback(uint16_t,GPIO_Pin){
//     if(GPIO_Pin == BUTTON_Pin){
//         uint32_t arr_value = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
//         uint32_t brightness =
//         (__HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_2)+100) % arr_value;
//         __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,brightness);
//
//         }
//     }
// }

//
//volatile uint8_t stop_flag = 1;// 默认停止
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    // 请将 BUTTON_Pin 替换为你实际的按键引脚名
//    if (GPIO_Pin == BUTTON_Pin) {
//        static uint32_t last_tick = 0;
//        uint32_t now = HAL_GetTick();
//        if (now - last_tick < 50) return;// 简单去抖 50ms
//        last_tick = now;
//        stop_flag ^= 1;// 切换 0/1
//    }
//}


// extern uint8_t rx_msg[4];

rc rc1;
// extern uint8_t rx_buffer[18];

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header_1;
extern CAN_TxHeaderTypeDef tx_header_2;
extern uint8_t tx_data_1[8];
extern uint8_t tx_data_2[8];
extern uint8_t rx_data[8];
extern Motor Motor_pitch;
extern Motor Motor_yaw;
extern uint32_t can_tx_mail_box_;
extern uint8_t stop_flag;
extern IMU imu;


// 新增：全局目标角度变量（可在其它模块通过 extern 访问/设置）
float target_angle = 45.0f;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    }
    
    // 处理电机反馈数据
    // if (rx_header.StdId == 0x201) {
    //     // M3508电机反馈（ID 0x201-0x208）
    //     Motor_pitch.canRxMsgCallback(rx_data);
    // } else if (rx_header.StdId >= 0x205 && rx_header.StdId <= 0x208) {
    //     // 其他电机反馈处理（可根据需要扩展）
    //     // 这里可以添加其他电机的处理逻辑
    // }
    if (rx_header.StdId == 0x208) {
        Motor_pitch.canRxMsgCallback(rx_data);
    } else if (rx_header.StdId == 0x205) {
        Motor_yaw.canRxMsgCallback(rx_data);
    }
    
    // 发送CAN接收事件（中断安全）
    // interrupt_notify.send_event_from_isr(INTERRUPT_EVENT_CAN_RX);
}

// void HAL_TIM_PeriodElapsedCallback(CAN_HandleTypeDef *htim) {
//   if (htim->Instance == htim6.Instance) {
//     HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
//   }
// }// 修正后的代码
void HAL_TIM_PeriodElapsedCallback(
        TIM_HandleTypeDef *htim) {// 1. 修正函数名和参数类型
    if (htim->Instance ==
        htim7.Instance) {// 2. 现在比较的都是 TIM_TypeDef*，类型正确

        Motor_pitch.handle();
        Motor_yaw.handle();
        HAL_CAN_AddTxMessage(&hcan1, &tx_header_1, tx_data_1, &can_tx_mail_box_);

        // 调用 Motor 对象的 handle 方法
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size) {
    if (huart == &huart3) {
        // 处理接收到的遥控器数据
        rc1.handle(rx_buffer);
        
        // // 发送遥控器数据就绪事件（中断安全）
        // interrupt_notify.send_event_from_isr(INTERRUPT_EVENT_RC_READY);
        
        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, 18);
        // osSemaphoreAcquire(rc_semaphore_handle,0);
    }
}
