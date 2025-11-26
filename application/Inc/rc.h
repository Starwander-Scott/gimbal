//
// Created by chenyincheng on 2025/10/25.
//

#ifndef C3_RC_H
#define C3_RC_H

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif
    extern uint8_t rx_buffer[18];
#ifdef __cplusplus
}
#endif




enum joystick{UP,MID,DOWN};

#ifdef __cplusplus







class rc {
private:
    uint8_t *rx_data_;
    float dead_zone_;  // 死区阈值
    
public:
    float channel0;
    float channel1;
    float channel2;
    float channel3;
    joystick s1;
    joystick s2;

    rc();
    void handle(uint8_t* rx_buffer);
    uint8_t *rx_buffer_;
    float linear_mapping(float x, uint16_t in_min, uint16_t in_max, float out_min, float out_max);
    //void calc();
    void init();
    
    // 新增功能：死区处理
    void set_dead_zone(float dead_zone);
    float apply_dead_zone(float value);
    
    // 拨杆状态检查
    bool is_system_enabled();  // 右拨杆最上档：系统使能
    bool is_system_disabled(); // 右拨杆最下档：系统失能
    bool is_emergency_stop();  // 紧急停止检查
    
    // 获取处理后的摇杆值（带死区）
    float get_pitch_input();   // 右摇杆上下：pitch控制
    float get_yaw_input();     // 右摇杆左右：yaw控制
};

extern rc rc1;

#endif
#endif // C3_RC_H