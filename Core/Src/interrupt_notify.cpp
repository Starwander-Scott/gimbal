#include "interrupt_notify.h"
#include "cmsis_os2.h"

// 定义事件队列大小
#define EVENT_QUEUE_SIZE 10

// 全局中断通知实例
InterruptNotify interrupt_notify;

InterruptNotify::InterruptNotify() : event_queue_(nullptr) {
}

bool InterruptNotify::init() {
    // 创建事件消息队列
    osMessageQueueAttr_t queue_attr = {
        .name = "InterruptEventQueue",
        .attr_bits = 0,
        .cb_mem = nullptr,
        .cb_size = 0,
        .mq_mem = nullptr,
        .mq_size = 0
    };
    
    event_queue_ = osMessageQueueNew(EVENT_QUEUE_SIZE, sizeof(InterruptEvent_t), &queue_attr);
    
    return (event_queue_ != nullptr);
}

bool InterruptNotify::send_event_from_isr(InterruptEvent_t event) {
    if (event_queue_ == nullptr) {
        return false;
    }
    
    // 使用中断安全的API发送消息
    osStatus_t status = osMessageQueuePut(event_queue_, &event, 0, 0);
    
    return (status == osOK);
}

bool InterruptNotify::receive_event(InterruptEvent_t* event, uint32_t timeout) {
    if (event_queue_ == nullptr || event == nullptr) {
        return false;
    }
    
    // 从队列接收事件
    osStatus_t status = osMessageQueueGet(event_queue_, event, nullptr, timeout);
    
    return (status == osOK);
}

uint32_t InterruptNotify::get_event_count() {
    if (event_queue_ == nullptr) {
        return 0;
    }
    
    return osMessageQueueGetCount(event_queue_);
}