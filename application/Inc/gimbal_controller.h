#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include "motor.h"
#include "imu.h"
#include "rc.h"

// 云台控制模式
typedef enum {
    GIMBAL_MODE_INIT = 0,
    GIMBAL_MODE_MANUAL,     // 手动遥控模式
    GIMBAL_MODE_AUTO,       // 自动模式
    GIMBAL_MODE_CALIBRATION // 校准模式
} GimbalMode_t;

// 云台控制器类
class GimbalController {
private:
    // 电机实例
    Motor pitch_motor;  // pitch轴电机
    Motor yaw_motor;    // yaw轴电机
    
    // IMU实例
    IMU* imu_sensor;
    
    // 控制参数
    GimbalMode_t control_mode;
    bool system_enabled;
    
    // 限位参数
    float pitch_min_limit;  // pitch轴最小角度限制
    float pitch_max_limit;  // pitch轴最大角度限制
    
    // 重力补偿参数
    float gravity_compensation[361];  // 0-360度的重力补偿值
    
public:
    GimbalController(float pitch_ratio, float yaw_ratio);
    
    // 初始化函数
    void init();
    
    // 主控制循环
    void control_loop();
    
    // 设置控制模式
    void set_control_mode(GimbalMode_t mode);
    
    // 系统使能/失能
    void enable_system();
    void disable_system();
    
    // 角度限制设置
    void set_pitch_limits(float min, float max);
    
    // 重力补偿相关函数
    void load_gravity_compensation(const float* compensation_table);
    float get_gravity_compensation(float angle);
    
    // 遥控器控制
    void process_remote_control(float pitch_input, float yaw_input);
    
    // 获取当前状态
    float get_pitch_angle();
    float get_yaw_angle();
    GimbalMode_t get_control_mode();
    bool is_system_enabled();
};

// 全局云台控制器实例
extern GimbalController gimbal_controller;

#endif // GIMBAL_CONTROLLER_H