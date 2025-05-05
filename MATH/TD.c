#include "td.h"

TD_State_t Move_X_TD;       // 小车前进线速度微分跟踪器
TD_State_t Car_Theta_TD;    // 小车航向角微分跟踪器

// 符号函数（静态内联，仅本文件使用）
static inline float sign_f32(float x) {
    if (x > 0.0f) return 1.0f;
    else if (x < 0.0f) return -1.0f;
    else return 0.0f;
}

// 最速综合函数fhan（静态封装，不暴露给外部）
static float fhan(float x1_error, float x2_derivative, float r, float h) {
    float d = r * h;
    float d0 = h * d;
    float y = x1_error + h * x2_derivative;

    // 计算平方根输入（添加保护）
    float sqrt_input = d * d + 8.0f * r * fabsf(y);
    if (sqrt_input < 0) sqrt_input = 0.0f; // 防止负数输入

    float a0;
    arm_sqrt_f32(sqrt_input, &a0);

    float a;
    if (fabsf(y) > d0) {
        a = x2_derivative + (a0 - d) * 0.5f * sign_f32(y);
    } else {
        // 避免除以零（h已在配置中确保不为零）
        a = x2_derivative + y / h;
    }

    if (fabsf(a) > d) {
        return -r * sign_f32(a);
    } else {
        return -r * a / d;
    }
}

/**
 * @brief 初始化跟踪微分器（TD）状态。
 * 
 * 此函数通过初始化跟踪微分器的内部变量和配置参数来设置其初始状态。
 * 
 * @param td 指向要初始化的 TD_State_t 结构体的指针。
 * @param r 跟踪微分器参数，定义收敛速度。
 * @param h 采样周期或步长。
 * @param max_out 跟踪微分器的最大输出限制。
 * 
 * @note 初始状态变量 x1 和 x2 被设置为 0。
 *       确保 `td` 指针有效并指向一个正确分配的结构体。
 */
void TD_Init(TD_State_t *td, float r, float h, float max_out) {
    td->x1 = 0;
    td->x2 = 0.0f;
    td->cfg.r = r; 
    td->cfg.h = h;
    td->cfg.max_out = max_out; // 输出限幅
}

// 更新跟踪器状态
void TD_Update(TD_State_t *td, float target) {
    float error = td->x1 - target;
    float u = fhan(error, td->x2, td->cfg.r, td->cfg.h);

    // 更新状态
    td->x1 += td->cfg.h * td->x2;
    td->x2 += td->cfg.h * u;

    // 输出限幅（可选）
    if (td->cfg.max_out > 0) {
        td->x1 = fmaxf(fminf(td->x1, td->cfg.max_out), -td->cfg.max_out);
    }
}

// 获取跟踪值
float TD_GetValue(const TD_State_t *td) {
    return td->x1;
}

// 获取微分值
float TD_GetDerivative(const TD_State_t *td) {
    return td->x2;
}

// 修改参数
void TD_SetConfig(TD_State_t *td, const TD_Config_t *cfg) {
    td->cfg = *cfg;
}