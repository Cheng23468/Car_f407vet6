#include "Odometry.h"

Diff_Odometry_t Diff_Car_Odometry;

// 初始化里程计
void Odometry_Init(Odometry_t *odom) {
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
    odom->distance = 0.0f;
}

// 更新里程计数据
void Odometry_Update(Odometry_t *odom, float v, float omega, float dt) {
    // 更新航向角（theta）
    odom->theta += omega * dt;

    // 确保 theta 在 [-pi, pi] 范围内
    if (odom->theta > PI) {
        odom->theta -= 2.0f * PI;
    } else if (odom->theta < -PI) {
        odom->theta += 2.0f * PI;
    }

    // 更新位置（x, y）
    odom->x += v * cosf(odom->theta) * dt;
    odom->y += v * sinf(odom->theta) * dt;

    // 更新累计行驶里程
    odom->distance += fabsf(v * dt);
}

// 获取里程计的当前位置
void Odometry_GetPosition(Odometry_t *odom, float *x, float *y, float *theta) {
    *x = odom->x;
    *y = odom->y;
    *theta = odom->theta;
}

// 获取里程计的累计行驶里程
float Odometry_GetDistance(Odometry_t *odom) {
    return odom->distance;
}

/**
 * @brief 初始化差速小车的里程计。
 * 
 * @param odom 指向要初始化的 Diff_Odometry_t 结构体的指针。
 * @param wheel_base 小车左右轮之间的距离（单位：米）。
 */
void DiffOdometry_Init(Diff_Odometry_t *odom, float wheel_base) {
    odom->left_wheel_speed = 0.0f;
    odom->right_wheel_speed = 0.0f;
    odom->wheel_base = wheel_base;

    // 调用里程计初始化
    Odometry_Init(&odom->Odometry);
}

/**
 * @brief 更新差速小车的里程计数据（融合 IMU 数据）。
 * 
 * 此函数通过使用提供的轮速和来自 IMU 的角速度，更新差速小车的里程计数据。
 * 它计算小车的线速度，并直接使用 IMU 提供的角速度，而不是通过轮速计算角速度。
 * 
 * @param odom 指向包含里程计数据的 Diff_Odometry_t 结构体的指针。
 * @param left_wheel_speed 左轮速度（单位：米/秒）。
 * @param right_wheel_speed 右轮速度（单位：米/秒）。
 * @param imu_angular_velocity 来自 IMU 的角速度（单位：弧度/秒）。
 * @param dt 更新的时间步长（单位：秒）。
 */
void DiffOdometry_UpdateWith_Wheel_IMU(Diff_Odometry_t *odom, float left_wheel_speed, float right_wheel_speed, float imu_angular_velocity, float dt) {
    // 更新左右轮速度
    odom->left_wheel_speed = left_wheel_speed;
    odom->right_wheel_speed = right_wheel_speed;

    // 计算小车的线速度
    float v = (left_wheel_speed + right_wheel_speed) / 2.0f;

    // 使用 IMU 提供的角速度代替计算的角速度
    float omega = imu_angular_velocity;

    // 调用里程计更新函数
    Odometry_Update(&odom->Odometry, v, omega, dt);
}

/**
 * @brief 仅使用IMU（加速度计和陀螺仪）更新里程计
 * 
 * @param odom 指向 Diff_Odometry_t 结构体的指针
 * @param linear_accel_x 车体坐标系下的x轴加速度（单位：m/s^2）
 * @param imu_angular_velocity IMU角速度（单位：rad/s）
 * @param dt 时间步长（单位：秒）
 */
void DiffOdometry_UpdateWith_IMU(Diff_Odometry_t *odom, float linear_accel_x, float imu_angular_velocity, float dt) {
    // 静态变量保存上一时刻的线速度
    static float last_linear_velocity = 0.0f;

    // 积分加速度得到线速度
    float linear_velocity = last_linear_velocity + linear_accel_x * dt;

    // 更新上一时刻速度
    last_linear_velocity = linear_velocity;

    // 只用IMU数据更新
    Odometry_Update(&odom->Odometry, linear_velocity, imu_angular_velocity, dt);
}

/**
 * @brief 更新差速小车的里程计数据（不使用 IMU 数据）。
 * 
 * 此函数通过左右轮速度计算小车的线速度和角速度，并更新里程计数据。
 * 
 * @param odom 指向包含里程计数据的 Diff_Odometry_t 结构体的指针。
 * @param left_wheel_speed 左轮速度（单位：米/秒）。
 * @param right_wheel_speed 右轮速度（单位：米/秒）。
 * @param dt 更新的时间步长（单位：秒）。
 */
void DiffOdometry_Update(Diff_Odometry_t *odom, float left_wheel_speed, float right_wheel_speed, float dt) {
    // 更新左右轮速度
    odom->left_wheel_speed = left_wheel_speed;
    odom->right_wheel_speed = right_wheel_speed;

    // 计算小车的线速度和角速度
    float v = (left_wheel_speed + right_wheel_speed) / 2.0f;
    float omega = (right_wheel_speed - left_wheel_speed) / odom->wheel_base;

    // 调用里程计更新函数
    Odometry_Update(&odom->Odometry, v, omega, dt);
}