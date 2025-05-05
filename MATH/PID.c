/**
  ******************************************************************************
  * File Name          : PID.c
  * Description        : 
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "PID.h"

PID_t Velocity_PID_A, Velocity_PID_B, Velocity_PID_C, Velocity_PID_D; // 电机速度环PID结构体
PID_t Car_Theta_PID; // 小车转向角PID结构体

/* Functions -----------------------------------------------------------------*/
/**
  * @brief 	PID控制器初始化
  * @retval None
  */
void PID_Init(PID_t *pid, float P, float I, float D, float Limit_Basis, float Limit_Output){
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->Limit_Basis = Limit_Basis;
    pid->Limit_Output = Limit_Output;
    pid->Basis = 0.0f;
    pid->Last = 0.0f;
    pid->Output = 0.0f;
}

/**
  * @brief 	PID控制器置零
  * @retval None
  */
void PID_SetZero(PID_t *pid){
    pid->Basis = 0.0f;
    pid->Last = 0.0f;
}

/**
  * @brief 	PID控制器计算
  * @retval float
  */
float PID_Generate(PID_t *pid, float Input, float Target){
    float Error = Target - Input;							
    pid->Basis += pid->I * Error;
    if(pid->Limit_Basis >= 0.0f){
        if(pid->Basis > pid->Limit_Basis){
            pid->Basis = pid->Limit_Basis;
        } else if(pid->Basis < -pid->Limit_Basis){
            pid->Basis = -pid->Limit_Basis;
        }
    }
    pid->Output = pid->P * Error + pid->Basis + pid->D * (Error - pid->Last);
    pid->Last = Error;
    if(pid->Limit_Output >= 0.0f){
        if(pid->Output > pid->Limit_Output){
            pid->Output = pid->Limit_Output;
        } else if(pid->Output < -pid->Limit_Output){
            pid->Output = -pid->Limit_Output;
        }
    }
    return pid->Output;
}

/**
  * @brief 	PID控制器圆周计算
  * @retval float
  */
float PID_GenerateRing(PID_t *pid, float Perimeter, float Input, float Target){
    float Error = Target - Input;
    if(Error > Perimeter / 2.0f){
        Error -= Perimeter;
    } else if(Error < -Perimeter / 2.0f){
        Error += Perimeter;
    }
    pid->Basis += pid->I * Error;
    if(pid->Limit_Basis >= 0.0f){
        if(pid->Basis > pid->Limit_Basis){
            pid->Basis = pid->Limit_Basis;
        } else if(pid->Basis < -pid->Limit_Basis){
            pid->Basis = -pid->Limit_Basis;
        }
    }
    pid->Output = pid->P * Error + pid->Basis + pid->D * (Error - pid->Last);
    pid->Last = Error;
    if(pid->Limit_Output >= 0.0f){
        if(pid->Output > pid->Limit_Output){
            pid->Output = pid->Limit_Output;
        } else if(pid->Output < -pid->Limit_Output){
            pid->Output = -pid->Limit_Output;
        }
    }
    return pid->Output;
}

/**
  * @brief 	PID控制器获取输出值
  * @retval None
  */
float PID_GetOutput(PID_t *pid){
    return pid->Output;
}
