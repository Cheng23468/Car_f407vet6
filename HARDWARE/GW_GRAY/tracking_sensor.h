#ifndef TRACKING_SENSOR_H
#define TRACKING_SENSOR_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "gw_gray_software_iic.h"
#include "PID.h"

#define TRACKING_SENSOR_LEFT_BLACK_POSITION     -50  // 传感器检测到左侧黑线路口时输出的位置
#define TRACKING_SENSOR_RIGHT_BLACK_POSITION    50   // 传感器检测到右侧黑线路口时输出的位置
#define TRACKING_SENSOR_BLACK_VALUE   100    // 检测状态时的黑线阈值
#define TRACKING_SENSOR_WHITE_VALUE   200   // 检测状态时的白线阈值

typedef enum {
    TrackingSensor_NormalLine = 0,  // 传感器正常检测到单根黑线
    TrackingSensor_AllBlack,    // 所有传感器都检测到黑线
    TrackingSensor_AllWhite,     // 所有传感器都检测到白线
    TrackingSensor_LeftBlack,   // 左侧传感器检测到黑线
    TrackingSensor_RightBlack,  // 右侧传感器检测到黑线
} TrackingSensor_State_t;

typedef struct
{
    // Public variables
    uint8_t sensor_values[8]; // iic获取的传感器值
    float position;           // 最终输出的传感器位置
    float target_position;    // 目标位置
    TrackingSensor_State_t state;   // 传感器检测到黑线的状态
    // Private variables
    float sensor_positions_const[8]; // 每个传感器的位置坐标 mm
    float calc_position;              // 计算的传感器位置 中间变量
    float last_position;             // 上次计算的位置
    float sigma;                     // 高斯权重分布的标准差
    float v_black;                   // 黑线理想值(阈值) 默认0
    uint8_t normalize_ntate;         // 传感器归一化状态
    PID_t PID;                       // 循迹PID控制器
} TrackingSensor_t;

extern TrackingSensor_t GrayTrackingSensor;

// Public functions
void TrackingSensor_init(TrackingSensor_t *Sensor, float sigma, float target_position);
void TrackingSensor_update(TrackingSensor_t *Sensor);

#ifdef __cplusplus
}
#endif
#endif // !TRACKING_SENSOR_H
