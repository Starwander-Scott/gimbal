//
// Created by chenyincheng on 2025/10/25.
//
#include "rc.h"
#include <stdint.h>
#include <cstring>
#include <cmath>

//从364-1684线性映射到-1-1

// 在 rc.h 或相应头文件中
uint8_t rx_buffer[18];
//临时定义一下，以免编译错误，待完善
extern rc rc1;


float rc::linear_mapping(float x, uint16_t in_min, uint16_t in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


rc::rc() {
    //  this->init();
    dead_zone_ = 0.1f;  // 默认死区阈值
}

void rc::init(){
    // memcpy(this->rx_data,rx_buffer,18);
    // this->parse_control_frame(this->rx_data);
    channel0 = 0.f;
    channel1 = 0.f;
    channel2 = 0.f;
    channel3 = 0.f;
    dead_zone_ = 0.1f;
}

// 设置死区阈值
void rc::set_dead_zone(float dead_zone) {
    dead_zone_ = dead_zone;
}

// 应用死区处理
float rc::apply_dead_zone(float value) {
    if (fabs(value) < dead_zone_) {
        return 0.0f;
    }
    return value;
}

// 拨杆状态检查
bool rc::is_system_enabled() {
    return s2 == UP;  // 右拨杆最上档：系统使能
}

bool rc::is_system_disabled() {
    return s2 == DOWN;  // 右拨杆最下档：系统失能
}

bool rc::is_emergency_stop() {
    return s2 == DOWN;  // 右拨杆最下档：紧急停止
}

// 获取处理后的摇杆值
float rc::get_pitch_input() {
    // 右摇杆上下控制pitch（channel2：上下）
    return apply_dead_zone(channel2);
}

float rc::get_yaw_input() {
    // 右摇杆左右控制yaw（channel3：左右）
    return apply_dead_zone(channel3);
}



// 高性能版本 - 直接位运算
void rc::handle(uint8_t* rx_data) {
    // 将前6个字节视为48位数据（实际使用前48位）
    rx_data_ = rx_buffer;
    uint64_t data = 0;
    for (int i = 0; i < 6; i++) {
        data |= ((uint64_t)rx_data[i]) << (8 * i);
    }

    // 提取各个位段
    uint16_t raw_ch0 = (data >> 0) & 0x7FF;  // 位0-10: 通道0
    uint16_t raw_ch1 = (data >> 11) & 0x7FF; // 位11-21: 通道1
    uint16_t raw_ch2 = (data >> 22) & 0x7FF; // 位22-32: 通道2
    uint16_t raw_ch3 = (data >> 33) & 0x7FF; // 位33-43: 通道3
    uint8_t raw_s1 = (data >> 44) & 0x03;    // 位44-45: S1
    uint8_t raw_s2 = (data >> 46) & 0x03;    // 位46-47: S2

    // 转换为实际物理值
    channel0 = linear_mapping(raw_ch0, 364, 1684, -1, 1);
    channel1 = linear_mapping(raw_ch1, 364, 1684, -1, 1);
    channel2 = linear_mapping(raw_ch2, 364, 1684, -1, 1);
    channel3 = linear_mapping(raw_ch3, 364, 1684, -1, 1);
    switch (raw_s1) {
        case 1:
            s1 = UP;
            break;
        case 2:
            s1 = DOWN;
            break;
        case 3:
            s1 = MID;
            break;

    }
    switch (raw_s2) {
        case 1:
            s2 = UP;
            break;
        case 2:
            s2 = DOWN;
            break;
        case 3:
            s2 = MID;
            break;
    }
}//
// Created by cycsjtuer on 2025/11/11.
//