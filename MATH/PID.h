/**
  ******************************************************************************
  * File Name          : PID.h
  * Description        : 
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

typedef struct {
    float P;	// ������ϵ��
    float I;	// ������ϵ��
    float D;	// ΢����ϵ��
    float Limit_Basis;		// �����޷�
    float Limit_Output;		// ����޷�
    float Basis;	// ����
    float Last;		// �ϴμ�������
    float Output;	// ���
} PID_t;

extern PID_t Velocity_PID_A, Velocity_PID_B, Velocity_PID_C, Velocity_PID_D;
extern PID_t Car_Theta_PID;

void PID_Init(PID_t *pid, float P, float I, float D, float Limit_Basis, float Limit_Output);
void PID_SetZero(PID_t *pid);
float PID_Generate(PID_t *pid, float Input, float Target);
float PID_GenerateRing(PID_t *pid, float Perimeter, float Input, float Target);
float PID_GetOutput(PID_t *pid);

#endif /* __PID_H */
