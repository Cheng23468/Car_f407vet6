#include "imu_task.h"

float acc_y_bias = 0.0f; // 加速度计y轴偏置
uint8_t bias_calc_flag = 0.0f; // 偏置计算标志

void imu_task(void *pvParameters)
{
    bias_calc_flag = 1;
    uint32_t lastWakeTime = getSysTickCnt();
    for (;;)
    {
        /* 此任务以200Hz的频率运行(5ms控制一次) */ 
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_200_HZ));

        if(bias_calc_flag) {
            IMU_RequestData(DM_IMU_CAN_ID, DM_IMU_ACCEL_REG);
            vTaskDelay(10);
            acc_y_bias = imu.accel[1];
            bias_calc_flag = 0;
        }
        if(0 != IMU_RequestData(DM_IMU_CAN_ID, DM_IMU_ACCEL_REG)) {   // 请求加速度数据
            Debug_Printf("CAN1 Send Failed!\r\n");
        }
        vTaskDelay(1);
        if(0 != IMU_RequestData(DM_IMU_CAN_ID, DM_IMU_GYRO_REG)) {   // 请求陀螺仪数据
            Debug_Printf("CAN1 Send Failed!\r\n");
        }
        vTaskDelay(1);
        if(0 != IMU_RequestData(DM_IMU_CAN_ID, DM_IMU_EULER_REG)) {   // 请求欧拉角数据
            Debug_Printf("CAN1 Send Failed!\r\n");
        }
        // DiffOdometry_UpdateWith_IMU(&Diff_Car_Odometry, imu.accel[1]-acc_y_bias, imu.gyro[2], 0.005f); // 更新里程计数据

        Debug_Printf("%f, %f, %f\r\n", imu.accel[1], imu.gyro[2], acc_y_bias);
    }
}