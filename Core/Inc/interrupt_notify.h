#ifndef INTERRUPT_NOTIFY_H
#define INTERRUPT_NOTIFY_H

#include "main.h"
#include "cmsis_os2.h"

// 中断通知事件定义
typedef enum {
    INTERRUPT_EVENT_CAN_RX = 0x01,     // CAN接收事件
    INTERRUPT_EVENT_RC_READY = 0x02,    // 遥控器数据就绪事件
    INTERRUPT_EVENT_EMERGENCY = 0x04,   // 紧急事件
} InterruptEvent_t;

// 中断通知管理器类
class InterruptNotify {
private:
    osMessageQueueId_t event_queue_;    // 事件消息队列
    
public:
    InterruptNotify();
    
    // 初始化中断通知系统
    bool init();
    
    // 从中断发送事件（中断安全）
    bool send_event_from_isr(InterruptEvent_t event);
    
    // 从任务接收事件
    bool receive_event(InterruptEvent_t* event, uint32_t timeout);
    
    // 获取事件队列状态
    uint32_t get_event_count();
};

// 全局中断通知实例
extern InterruptNotify interrupt_notify;

#endif // INTERRUPT_NOTIFY_H