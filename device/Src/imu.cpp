
//
// Created by cycsjtuer on 2025/11/11.
//

#include "imu.h"
#include "bmi088.h"
#include "main.h"
#include <cmath>
#include <cstring>

// 常量定义
static constexpr float g_const = 9.80665f;
static constexpr float PI      = 3.14159265358979323846f;

static inline float deg2rad(float x) { return x * PI / 180.0f; }
static inline float rad2deg(float x) { return x * 180.0f / PI; }

// BMI088 量程配置（与 bmi088.c 中一致）
#define BMI088_ACCEL_RANGE_3G   0x00
#define BMI088_ACCEL_RANGE_6G   0x01
#define BMI088_ACCEL_RANGE_12G  0x02
#define BMI088_ACCEL_RANGE_24G  0x03

#define BMI088_GYRO_RANGE_2000DPS 0x00
#define BMI088_GYRO_RANGE_1000DPS 0x01
#define BMI088_GYRO_RANGE_500DPS  0x02
#define BMI088_GYRO_RANGE_250DPS  0x03
#define BMI088_GYRO_RANGE_125DPS  0x04

//==================== IMU 类实现 ====================

IMU::IMU(const float& dt, const float& kg, const float& g_thres,
         const float R_imu[3][3], const float gyro_bias[3])
  : mahony_(dt, kg, g_thres),   // Mahony 构造
    euler_deg_(), euler_rad_()  // 默认 0
{
  // 保存解算参数
  std::memcpy(R_imu_, R_imu, sizeof(R_imu_));
  std::memcpy(gyro_bias_, gyro_bias, sizeof(gyro_bias_));

  // 初始化状态
  std::memset(&raw_data_, 0, sizeof(raw_data_));
  std::memset(gyro_sensor_, 0, sizeof(gyro_sensor_));
  std::memset(gyro_world_, 0, sizeof(gyro_world_));
  std::memset(gyro_sensor_dps_, 0, sizeof(gyro_sensor_dps_));
  std::memset(gyro_world_dps_, 0, sizeof(gyro_world_dps_));
  std::memset(accel_sensor_, 0, sizeof(accel_sensor_));
  std::memset(accel_world_, 0, sizeof(accel_world_));
  q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
}

IMU::IMU()
  : IMU(0.001f, 0.1f, 0.1f,            // 默认 dt=1ms, kg=0.1, g_thres=0.1
        (const float[3][3]){{1,0,0},{0,1,0},{0,0,1}},
        (const float[3]){0.f,0.f,0.f}) {}

// 初始化 BMI088 并设置初始姿态
void IMU::init(EulerAngle_t euler_deg_init) {
  bmi088_init();

  // 记录初始欧拉角（角度、弧度）
  euler_deg_ = euler_deg_init;
  euler_rad_.yaw   = deg2rad(euler_deg_init.yaw);
  euler_rad_.pitch = deg2rad(euler_deg_init.pitch);
  euler_rad_.roll  = deg2rad(euler_deg_init.roll);

  // 根据欧拉角生成初始四元数（Z-Y-X，yaw-pitch-roll）
  const float cy = std::cos(euler_rad_.yaw   * 0.5f);
  const float sy = std::sin(euler_rad_.yaw   * 0.5f);
  const float cp = std::cos(euler_rad_.pitch * 0.5f);
  const float sp = std::sin(euler_rad_.pitch * 0.5f);
  const float cr = std::cos(euler_rad_.roll  * 0.5f);
  const float sr = std::sin(euler_rad_.roll  * 0.5f);

  q_[0] = cr * cp * cy + sr * sp * sy;
  q_[1] = sr * cp * cy - cr * sp * sy;
  q_[2] = cr * sp * cy + sr * cp * sy;
  q_[3] = cr * cp * sy - sr * sp * cy;
}

// 读取 BMI088 传感器原始数据并转换为物理单位
void IMU::readSensor() {
  uint8_t raw_range;
  int16_t raw_acc[3];
  int16_t raw_gyro[3];

  //---------- 加速度 ----------
  bmi088_accel_read_reg(0x41, &raw_range, 1);
  float acc_range_factor;
  switch (raw_range & 0x03) {
    case BMI088_ACCEL_RANGE_3G:  acc_range_factor = 3.0f  / 32768.0f; break;
    case BMI088_ACCEL_RANGE_6G:  acc_range_factor = 6.0f  / 32768.0f; break;
    case BMI088_ACCEL_RANGE_12G: acc_range_factor = 12.0f / 32768.0f; break;
    case BMI088_ACCEL_RANGE_24G: acc_range_factor = 24.0f / 32768.0f; break;
    default:                     acc_range_factor = 3.0f  / 32768.0f; break;
  }
  bmi088_accel_read_reg(0x12, rx_acc_data, 6);
  for (int i = 0; i < 3; ++i) {
    raw_acc[i] = static_cast<int16_t>((rx_acc_data[2*i+1] << 8) | rx_acc_data[2*i]);
    // 保存到原始数据结构（单位 g）
    raw_data_.accel[i] = raw_acc[i] * acc_range_factor;
    // 转为 m/s^2 存到 accel_sensor_
    accel_sensor_[i] = raw_data_.accel[i] * g_const;
  }

  //---------- 陀螺仪 ----------
  bmi088_gyro_read_reg(0x0F, &raw_range, 1);
  float gyro_range_factor;
  switch (raw_range & 0x07) {
    case BMI088_GYRO_RANGE_2000DPS: gyro_range_factor = 2000.0f / 32768.0f; break;
    case BMI088_GYRO_RANGE_1000DPS: gyro_range_factor = 1000.0f / 32768.0f; break;
    case BMI088_GYRO_RANGE_500DPS:  gyro_range_factor = 500.0f  / 32768.0f; break;
    case BMI088_GYRO_RANGE_250DPS:  gyro_range_factor = 250.0f  / 32768.0f; break;
    case BMI088_GYRO_RANGE_125DPS:  gyro_range_factor = 125.0f  / 32768.0f; break;
    default:                        gyro_range_factor = 2000.0f / 32768.0f; break;
  }
  bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);
  for (int i = 0; i < 3; ++i) {
    raw_gyro[i] = static_cast<int16_t>((rx_gyro_data[2*i+1] << 8) | rx_gyro_data[2*i]);
    // 原始角速度 dps（减去零偏）
    gyro_sensor_dps_[i] = raw_gyro[i] * gyro_range_factor - gyro_bias_[i];
    // 转为 rad/s
    gyro_sensor_[i] = deg2rad(gyro_sensor_dps_[i]);
    // 保存到原始数据结构
    raw_data_.gyro[i] = gyro_sensor_dps_[i];
  }

  // 温度（如果需要，可从 BMI088 读取，这里先置 0）
  raw_data_.temp[0] = 0.0f;
}

// // 姿态更新：Mahony 融合，更新 q\_ / euler\_
// void IMU::update(void) {
//   // Mahony 输入：当前四元数、角速度(rad/s)、加速度(m/s^2)
//   mahony_.update(q_, gyro_sensor_, accel_sensor_);
//
//   // 四元数 -> 欧拉角（Z-Y-X，yaw-pitch-roll）
//   const float qw = q_[0];
//   const float qx = q_[1];
//   const float qy = q_[2];
//   const float qz = q_[3];
//
//   // 参考常见转换公式
//   float sinr_cosp = 2.0f * (qw * qx + qy * qz);
//   float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
//   float roll  = std::atan2(sinr_cosp, cosr_cosp);
//
//   float sinp = 2.0f * (qw * qy - qz * qx);
//   float pitch;
//   if (std::fabs(sinp) >= 1.0f)
//     pitch = std::copysign(PI / 2.0f, sinp);
//   else
//     pitch = std::asin(sinp);
//
//   float siny_cosp = 2.0f * (qw * qz + qx * qy);
//   float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
//   float yaw   = std::atan2(siny_cosp, cosy_cosp);
//
//   euler_rad_.roll  = roll;
//   euler_rad_.pitch = pitch;
//   euler_rad_.yaw   = yaw;
//
//   euler_deg_.roll  = rad2deg(roll);
//   euler_deg_.pitch = rad2deg(pitch);
//   euler_deg_.yaw   = rad2deg(yaw);
// }

// 这里简单返回 1，表示“有数据”
int IMU::got_data() {
  return 1;
}

// 可供外部 C 代码使用的兼容接口（如需）
extern "C" {

void* imu_create(void) {
  auto* imu = new IMU();
  return static_cast<void*>(imu);
}

void imu_destroy(void* imu_ptr) {
  auto* imu = static_cast<IMU*>(imu_ptr);
  delete imu;
}

float* imu_acc_calculate(void* imu_ptr) {
  auto* imu = static_cast<IMU*>(imu_ptr);
  imu->readSensor();
  return imu->getAccel();   // 如无单独实现，可直接返回 accel_sensor\_
}

float* imu_gyro_calculate(void* imu_ptr) {
  auto* imu = static_cast<IMU*>(imu_ptr);
  imu->readSensor();
  return imu->getGyro() ;  // 如无单独实现，可直接返回 gyro_sensor\_
}

int imu_got_data(void* imu_ptr) {
  auto* imu = static_cast<IMU*>(imu_ptr);
  return imu->got_data();
}

} // extern "C"


































// //
// // Created by cycsjtuer on 2025/11/11.
// //
//
// #include "imu.h"
// #include "main.h"
// #include <iostream>
// #include <cmath>
//
//
// // 假设的常量定义，实际值需参考最新数据手册
// #define g 9.80665f // 重力加速度
// #define pi 3.14159265358979323846f // 圆周率
// #define deg2rad(x) (x * pi / 180.0f) // 角度转弧度
// #define BMI088_ACCEL_RANGE_3G 0x00
// #define BMI088_ACCEL_RANGE_6G 0x01
// #define BMI088_ACCEL_RANGE_12G 0x02
// #define BMI088_ACCEL_RANGE_24G 0x03
//
// #define BMI088_GYRO_RANGE_2000DPS 0x00
// #define BMI088_GYRO_RANGE_1000DPS 0x01
// #define BMI088_GYRO_RANGE_500DPS 0x02
// #define BMI088_GYRO_RANGE_250DPS 0x03
// #define BMI088_GYRO_RANGE_125DPS 0x04
//
// #define BMI088_GYRO_BW_532HZ 0x00
// #define BMI088_GYRO_BW_230HZ 0x01
// #define BMI088_GYRO_BW_116HZ 0x02
// #define BMI088_GYRO_BW_47HZ  0x03
// #define BMI088_GYRO_BW_23HZ  0x04
// #define BMI088_GYRO_BW_12HZ  0x05
// #define BMI088_GYRO_BW_64HZ  0x06
// #define BMI088_GYRO_BW_32HZ  0x07
//
// extern void bmi088_accel_read_reg(uint8_t reg_addr, uint8_t *data, uint8_t len);
// extern void bmi088_gyro_read_reg(uint8_t reg_addr, uint8_t *data, uint8_t len);
// extern void bmi088_accel_write_reg(uint8_t reg_addr, uint8_t data);
// extern void bmi088_gyro_write_reg(uint8_t reg_addr, uint8_t data);
//
// //float acceleration[] = {0, 0, 0};
// //float gyro[] = {0, 0, 0};
//
// // IMU::IMU() {
// //   std::cout << "IMU init" << std::endl;
// // }
// //
// // IMU::~IMU() {
// //   std::cout << "IMU deinit" << std::endl;
// // }
//
// int IMU::got_data() {
//   return 1;
// }
//
// float* IMU::acc_calculate() {
//   uint8_t raw_range;
//
//   float acc_range_factor; // 量程系数
//   int16_t raw_acc[3];     // 三轴加速度原始值
//
//
//   // 1. 读取0x41寄存器中的量程参数，并换算为量程系数
//   bmi088_accel_read_reg(0x41, &raw_range, 1);
//
//   // 提取量程配置位（寄存器低2位）
//   uint8_t range_config = raw_range & 0x03;
//
//   // 根据量程配置设置量程系数
//   switch (range_config) {
//   case BMI088_ACCEL_RANGE_3G:
//     acc_range_factor = 3.0f / 32768.0f; // 16位有符号数 -> ±32768
//     break;
//   case BMI088_ACCEL_RANGE_6G:
//     acc_range_factor = 6.0f / 32768.0f;
//     break;
//   case BMI088_ACCEL_RANGE_12G:
//     acc_range_factor = 12.0f / 32768.0f;
//     break;
//   case BMI088_ACCEL_RANGE_24G:
//     acc_range_factor = 24.0f / 32768.0f;
//     break;
//   default:
//     // 默认使用3G量程
//     acc_range_factor = 3.0f / 32768.0f;
//     break;
//   }
//
//   // 2. 读取0x12寄存器开始的6字节加速度数据
//   bmi088_accel_read_reg(0x12, rx_acc_data, 6);
//
//   // 3. 将两个8位数据组合成16位有符号整数（注意字节顺序）
//   for (int i = 0; i < 3; i++) {
//     // BMI088通常采用高字节在前的方式[10](@ref)
//     raw_acc[i] = (int16_t)((rx_acc_data[2*i+1] << 8) | rx_acc_data[2*i]);
//   }
//
//   // 4. 用量程系数将原始数据转换为加速度值(g)
//   for (int i = 0; i < 3; i++) {
//     accel_sensor_[i] = raw_acc[i] * acc_range_factor * g;
//   }
//   return accel_sensor_;
//       //可添加进一步处理，如传感器校准、滤波等
// }
//
//
// void IMU::readSensor() {
//
//
//
// }
//
//
//
//
//
// //void IMU::gyro_calculate(){
//   // 1. 设置/读取gyro0x0F寄存器中的量程range参数，并换算为量程系数
//
//   // 2. 读取gyro0x02寄存器中的6位gyro数据
//   // 3. 用量程系数将原始数据转换为常用单位
// //}
//
//
// float* IMU::gyro_calculate() {
//   uint8_t raw_range;
//
//   float gyro_range_factor; // 量程系数
//   int16_t raw_gyro[3];     // 三轴陀螺仪原始值
//   float gyro[3];           // 转换后的角速度值，单位为dps
//
//   // 1. 读取0x0F寄存器中的量程参数
//   bmi088_gyro_read_reg(0x0F, &raw_range, 1);
//
//   // 提取量程配置位（寄存器低3位）
//   uint8_t range_config = raw_range & 0x07;
//
//   // 根据量程配置设置量程系数
//   switch (range_config) {
//   case BMI088_GYRO_RANGE_2000DPS:
//     gyro_range_factor = 2000.0f / 32768.0f;
//     break;
//   case BMI088_GYRO_RANGE_1000DPS:
//     gyro_range_factor = 1000.0f / 32768.0f;
//     break;
//   case BMI088_GYRO_RANGE_500DPS:
//     gyro_range_factor = 500.0f / 32768.0f;
//     break;
//   case BMI088_GYRO_RANGE_250DPS:
//     gyro_range_factor = 250.0f / 32768.0f;
//     break;
//   case BMI088_GYRO_RANGE_125DPS:
//     gyro_range_factor = 125.0f / 32768.0f;
//     break;
//   default:
//     // 默认使用2000DPS量程
//     gyro_range_factor = 2000.0f / 32768.0f;
//     break;
//   }
//
//   // 2. 读取0x02寄存器开始的6字节陀螺仪数据
//   bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);
//
//   // 3. 将两个8位数据组合成16位有符号整数
//   for (int i = 0; i < 3; i++) {
//     raw_gyro[i] = (int16_t)((rx_gyro_data[2*i+1] << 8) | rx_gyro_data[2*i]);
//   }
//
//   // 4. 用量程系数将原始数据转换为角速度值(dps)
//   for (int i = 0; i < 3; i++) {
//     gyro[i] = raw_gyro[i] * gyro_range_factor;
//   }
//   return gyro;
//   // 此处可添加进一步处理，如传感器校准、滤波等
// }
//
//
// // 在imu.cpp文件末尾添加以下实现
// extern "C" {
//
// void* imu_get_handle(void) {
//   static IMU imu;
//   return &imu;
// }
//
// void* imu_create(void) {
//   return new IMU();
// }
//
// // 销毁IMU对象
// void imu_destroy(void* imu_ptr) {
//   IMU* imu = static_cast<IMU*>(imu_ptr);
//   delete imu;
// }
//
// // 调用加速度计算函数
// float* imu_acc_calculate(void* imu_ptr) {
//   IMU* imu = static_cast<IMU*>(imu_ptr);
//   return imu->acc_calculate();
// }
//
// // 调用陀螺仪计算函数
// float* imu_gyro_calculate(void* imu_ptr) {
//   IMU* imu = static_cast<IMU*>(imu_ptr);
//   return imu->gyro_calculate();
// }
//
// // 调用数据获取状态函数
// int imu_got_data(void* imu_ptr) {
//   IMU* imu = static_cast<IMU*>(imu_ptr);
//   return imu->got_data();
// }
//
// } // extern "C"