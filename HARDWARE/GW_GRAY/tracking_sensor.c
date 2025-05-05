#include "tracking_sensor.h"

TrackingSensor_t GrayTrackingSensor; // 灰度循迹传感器实例

/**
 * @brief 初始化灰度循迹传感器
 *
 * 此函数用于初始化灰度循迹传感器的相关参数，包括传感器位置、PID控制器以及归一化模式。
 *
 * @param Sensor 指向TrackingSensor_t结构体的指针，用于存储传感器的相关信息。
 * @param sigma 传感器位置计算的标准差，用于计算权重。
 * @param target_position 目标位置，用于循迹控制。
 *
 * 初始化内容包括：
 * - 设置传感器的固定位置数组（sensor_positions_const）。
 * - 初始化传感器的当前值数组（sensor_values）。
 * - 设置初始位置为0（last_position）。
 * - 设置传感器的标准差（sigma）。
 * - 初始化黑色阈值（v_black）。
 * - 设置目标位置（target_position）。
 * - 初始化PID控制器（PID）。
 * - 开启灰度循迹模块的归一化模式（IIC_Turn_On_Normalize）。
 */
void TrackingSensor_init(TrackingSensor_t *Sensor, float sigma, float target_position)
{
    // 初始化传感器位置
    float _positions[8] = {-42, -30, -18, -6, 6, 18, 30, 42};
    for (int i = 0; i < 8; i++)
    {
        Sensor->sensor_positions_const[i] = _positions[i];
        Sensor->sensor_values[i] = 0;
    }
    Sensor->position = 0.0f;
    Sensor->calc_position = 0.0f;
    Sensor->last_position = 0.0f;
    Sensor->sigma = sigma;
    Sensor->v_black = 0;
    Sensor->target_position = target_position;
    Sensor->PID = (PID_t){0}; // 初始化PID控制器
    IIC_Turn_On_Normalize();  // 开启灰度循迹模块归一化模式
}

/**
 * @brief 计算灰度循迹传感器的位置
 *
 * 此函数通过传感器的当前值和高斯权重计算传感器的位置。
 *
 * @param Sensor 指向TrackingSensor_t结构体的指针，用于存储传感器的相关信息。
 * @return float 返回计算得到的传感器位置。
 *
 * 计算过程包括：
 * - 根据传感器的当前值和黑色阈值计算高斯权重。
 * - 使用高斯权重计算加权和。
 * - 根据加权和和权重总和计算传感器的位置。
 */
float TrackingSensor_calculate_position(TrackingSensor_t *Sensor)
{
    float weights[8], sum_weights = 0, weighted_sum = 0;
    // 计算高斯权重
    for (int i = 0; i < 8; i++)
    {
        float diff = Sensor->sensor_values[i] - Sensor->v_black;
        weights[i] = expf(-(diff * diff) / (2 * Sensor->sigma * Sensor->sigma)); // 高斯函数
        sum_weights += weights[i];
        weighted_sum += Sensor->sensor_positions_const[i] * weights[i];
    }

    Sensor->calc_position = weighted_sum / sum_weights;
    return Sensor->calc_position;
}

void TrackingSensor_update(TrackingSensor_t *Sensor)
{
    // 判断传感器是否开启归一化，如否则开启归一化
    if (IIC_Get_Normalize(&Sensor->normalize_ntate, 1))
    {
        if (Sensor->normalize_ntate != 0xFF)
        {
            IIC_Turn_On_Normalize();
        }
    }
    // 更新传感器值
    IIC_Get_Anolog(Sensor->sensor_values, 8);
    Sensor->last_position = Sensor->calc_position;

    // 左半边黑线判断
    if (Sensor->sensor_values[0] < TRACKING_SENSOR_BLACK_VALUE &&
        Sensor->sensor_values[1] < TRACKING_SENSOR_BLACK_VALUE &&
        Sensor->sensor_values[2] < TRACKING_SENSOR_BLACK_VALUE &&
        // Sensor->sensor_values[5] > TRACKING_SENSOR_WHITE_VALUE &&
        Sensor->sensor_values[6] > TRACKING_SENSOR_WHITE_VALUE &&
        Sensor->sensor_values[7] > TRACKING_SENSOR_WHITE_VALUE)
    {
        Sensor->state = TrackingSensor_LeftBlack;
        Sensor->position = TRACKING_SENSOR_LEFT_BLACK_POSITION;
    }
    // 右半边黑线判断
    else if (Sensor->sensor_values[0] > TRACKING_SENSOR_WHITE_VALUE &&
             Sensor->sensor_values[1] > TRACKING_SENSOR_WHITE_VALUE &&
            //  Sensor->sensor_values[2] > TRACKING_SENSOR_WHITE_VALUE &&
             Sensor->sensor_values[5] < TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[6] < TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[7] < TRACKING_SENSOR_BLACK_VALUE)
    {
        Sensor->state = TrackingSensor_RightBlack;
        Sensor->position = TRACKING_SENSOR_RIGHT_BLACK_POSITION;
    }
    // 全为黑线判断
    else if (Sensor->sensor_values[0] < TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[1] < TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[2] < TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[3] < TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[4] < TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[5] < TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[6] < TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[7] < TRACKING_SENSOR_BLACK_VALUE)
    {
        Sensor->state = TrackingSensor_AllBlack;
        Sensor->position = 0; // 全黑线时位置为0
    }
    // 未检测到黑线判断 包括两边传感器的临界状态
    else if (Sensor->sensor_values[0] > TRACKING_SENSOR_BLACK_VALUE &&
             Sensor->sensor_values[1] > TRACKING_SENSOR_WHITE_VALUE &&
             Sensor->sensor_values[2] > TRACKING_SENSOR_WHITE_VALUE &&
             Sensor->sensor_values[3] > TRACKING_SENSOR_WHITE_VALUE &&
             Sensor->sensor_values[4] > TRACKING_SENSOR_WHITE_VALUE &&
             Sensor->sensor_values[5] > TRACKING_SENSOR_WHITE_VALUE &&
             Sensor->sensor_values[6] > TRACKING_SENSOR_WHITE_VALUE &&
             Sensor->sensor_values[7] > TRACKING_SENSOR_BLACK_VALUE)
    {
        Sensor->state = TrackingSensor_AllWhite;
        Sensor->position = Sensor->last_position; // 继续使用上次位置
    }
    // 正常检测到单根黑线判断
    else
    {
        Sensor->state = TrackingSensor_NormalLine;
        Sensor->calc_position = TrackingSensor_calculate_position(Sensor); // 计算传感器位置
        Sensor->position = Sensor->calc_position;                          // 更新传感器位置}
    }
}