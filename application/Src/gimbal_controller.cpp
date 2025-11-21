#include "gimbal_controller.h"
#include "math.h"
#include "cmsis_os2.h"

// 全局云台控制器实例
GimbalController gimbal_controller(36.0f, 1.0f); // pitch减速比36:1，yaw减速比1:1

GimbalController::GimbalController(float pitch_ratio, float yaw_ratio) 
    : pitch_motor(pitch_ratio), yaw_motor(yaw_ratio),
      imu_sensor(nullptr), control_mode(GIMBAL_MODE_INIT),
      system_enabled(false), pitch_min_limit(-30.0f), pitch_max_limit(60.0f) {
    // 初始化重力补偿表（默认值）
    for (int i = 0; i <= 360; i++) {
        gravity_compensation[i] = 0.0f;
    }
}

void GimbalController::init() {
    // 初始化电机
    // pitch_motor.init();
    // yaw_motor.init();
    
    // 设置初始控制模式
    control_mode = GIMBAL_MODE_MANUAL;
    system_enabled = false; // 默认失能
    
    // 设置角度限制
    set_pitch_limits(pitch_min_limit, pitch_max_limit);
}

void GimbalController::control_loop() {
    if (!system_enabled) {
        // 系统失能时停止所有电机
        pitch_motor.Motor_Stop();
        yaw_motor.Motor_Stop();
        return;
    }
    
    switch (control_mode) {
        case GIMBAL_MODE_MANUAL:
            // 手动模式：遥控器控制
            // 遥控器输入处理在process_remote_control中完成
            break;
            
        case GIMBAL_MODE_AUTO:
            // 自动模式：根据IMU或其他传感器自动控制
            // TODO: 实现自动控制逻辑
            break;
            
        case GIMBAL_MODE_CALIBRATION:
            // 校准模式
            // TODO: 实现校准逻辑
            break;
            
        default:
            break;
    }
    
    // 处理电机控制
    pitch_motor.handle();
    yaw_motor.handle();
}

void GimbalController::set_control_mode(GimbalMode_t mode) {
    control_mode = mode;
}

void GimbalController::enable_system() {
    system_enabled = true;
}

void GimbalController::disable_system() {
    system_enabled = false;
    // 立即停止电机
    pitch_motor.Motor_Stop();
    yaw_motor.Motor_Stop();
}

void GimbalController::set_pitch_limits(float min, float max) {
    pitch_min_limit = min;
    pitch_max_limit = max;
}

void GimbalController::load_gravity_compensation(const float* compensation_table) {
    for (int i = 0; i <= 360; i++) {
        gravity_compensation[i] = compensation_table[i];
    }
}

float GimbalController::get_gravity_compensation(float angle) {
    // 角度归一化到0-360度
    angle = fmod(angle, 360.0f);
    if (angle < 0) angle += 360.0f;
    
    int index = static_cast<int>(angle);
    if (index < 0) index = 0;
    if (index > 360) index = 360;
    
    return gravity_compensation[index];
}

void GimbalController::process_remote_control(float pitch_input, float yaw_input) {
    if (!system_enabled) return;
    
    // 死区处理（防止摇杆漂移）
    const float dead_zone = 0.1f;
    if (fabs(pitch_input) < dead_zone) pitch_input = 0.0f;
    if (fabs(yaw_input) < dead_zone) yaw_input = 0.0f;
    
    // 映射遥控器输入到目标角度（示例映射）
    float pitch_target = pitch_input * 30.0f; // ±30度范围
    float yaw_target = yaw_input * 180.0f;     // ±180度范围
    
    // 角度限制检查
    if (pitch_target < pitch_min_limit) pitch_target = pitch_min_limit;
    if (pitch_target > pitch_max_limit) pitch_target = pitch_max_limit;
    
    // 获取重力补偿
    float gravity_ff = get_gravity_compensation(pitch_motor.getCurrentAngle());
    
    // 设置电机目标位置
    pitch_motor.SetPosition(pitch_target, 0.0f, gravity_ff);
    yaw_motor.SetPosition(yaw_target, 0.0f, 0.0f);
}

float GimbalController::get_pitch_angle() {
    return pitch_motor.getCurrentAngle();
}

float GimbalController::get_yaw_angle() {
    return yaw_motor.getCurrentAngle();
}

GimbalMode_t GimbalController::get_control_mode() {
    return control_mode;
}

bool GimbalController::is_system_enabled() {
    return system_enabled;
}