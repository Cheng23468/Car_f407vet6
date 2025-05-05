/*
 * FeedForward.cpp
 *
 *  Created on: Nov 25, 2024
 *      Author: Cheng
 */

#include "FeedForward.h"

FeedForward_t Velocity_FF_A, Velocity_FF_B; // 电机速度环前馈控制器结构体

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
void FeedForward_Init(FeedForward_t* ff, float Ka, float Kb, float Kc, float T) {
    ff->Ka = Ka;
    ff->Kb = Kb;
    ff->Kc = Kc;
    ff->T = T;
    ff->Output = 0.0f;
    ff->LastInput = 0.0f;
    ff->LastInput2 = 0.0f;
}

float FeedForward_Generate(FeedForward_t* ff, float Input) {
    ff->Output = ff->Ka * Input
               + ff->Kb * (Input - ff->LastInput) / ff->T
               + ff->Kc * (Input - 2 * ff->LastInput + ff->LastInput2) / (ff->T * ff->T);
    ff->LastInput2 = ff->LastInput;
    ff->LastInput = Input;
    return ff->Output;
}

float FeedForward_GetOutput(FeedForward_t* ff) {
    return ff->Output;
}
