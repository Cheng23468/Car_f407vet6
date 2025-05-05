#ifndef TD_H
#define TD_H
// 微分跟踪器
#include "arm_math.h"  // 依赖ARM数学库
#include <math.h>

// 微分跟踪器参数配置结构体
typedef struct {
    float r;      // 快速因子（默认1.0）
    float h;      // 步长（必须大于0）
    float max_out;// 输出限幅（可选）
} TD_Config_t;

// 微分跟踪器状态结构体
typedef struct {
    float x1;     // 跟踪的目标值（平滑后）
    float x2;     // 目标值的微分
    TD_Config_t cfg; // 当前配置参数
} TD_State_t;

extern TD_State_t Move_X_TD;
extern TD_State_t Car_Theta_TD;

void TD_Init(TD_State_t *td, float r, float h, float max_out);
void TD_Update(TD_State_t *td, float target);
float TD_GetValue(const TD_State_t *td);
float TD_GetDerivative(const TD_State_t *td);
void TD_SetConfig(TD_State_t *td, const TD_Config_t *cfg);

#endif