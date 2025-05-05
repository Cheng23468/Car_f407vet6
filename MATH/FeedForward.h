/*
 * FeedForward.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Cheng
 */

#ifndef SRC_FEEDFORWARD_H_
#define SRC_FEEDFORWARD_H_

/*
 * @brief 前馈控制器
 * 采样周期：T 单位s
 * 电机转速/电流：G(s) = m / (n*s^2 + p*s + q)
 * 前馈：Gf(s) = q/m + p/m * s + 1/m * s^2
 * 			 = Ka + Kb*s + Kc*s^2
 *
 * 输出：output = Ka*input + Kb*(input - lastInput)/T + Kc*(input - 2*lastInput + lastInput2)/T^2
 *
 */
typedef struct {
    float T;             // 采样周期
    float Ka;            // 系数a
    float Kb;            // 系数b
    float Kc;            // 系数c

    float Output;        // 输出
    float LastInput;     // 上次输入
    float LastInput2;    // 上上次输入
} FeedForward_t;

extern FeedForward_t Velocity_FF_A, Velocity_FF_B;

void FeedForward_Init(FeedForward_t* ff, float Ka, float Kb, float Kc, float T);
float FeedForward_Generate(FeedForward_t* ff, float Input);
float FeedForward_GetOutput(FeedForward_t* ff);


#endif /* SRC_FEEDFORWARD_H_ */
