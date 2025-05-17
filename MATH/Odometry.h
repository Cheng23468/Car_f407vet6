#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "arm_math.h"

// 里程计
typedef struct {
    float x;          // 小车在 X 轴上的位置（单位：米）
    float y;          // 小车在 Y 轴上的位置（单位：米）
    float theta;      // 小车的航向角（单位：弧度）
    float distance;   // 小车累计行驶的总里程（单位：米）
} Odometry_t;

// 差速小车里程计结构体
typedef struct {
    float left_wheel_speed;  // 左轮速度（单位：米/秒）
    float right_wheel_speed; // 右轮速度（单位：米/秒）
    float wheel_base;     // 车轮之间的距离（单位：米）
    Odometry_t Odometry; // 里程计数据
} Diff_Odometry_t;

extern Diff_Odometry_t Diff_Car_Odometry;

void DiffOdometry_Init(Diff_Odometry_t *odom, float wheel_base);
void DiffOdometry_UpdateWith_Wheel_IMU(Diff_Odometry_t *odom, float left_wheel_speed, float right_wheel_speed, float imu_angular_velocity, float dt);
void DiffOdometry_UpdateWith_IMU(Diff_Odometry_t *odom, float linear_accel_x, float imu_angular_velocity, float dt);
void DiffOdometry_Update(Diff_Odometry_t *odom, float left_wheel_speed, float right_wheel_speed, float dt);

#endif // ODOMETRY_H