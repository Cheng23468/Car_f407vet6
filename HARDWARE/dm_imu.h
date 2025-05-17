#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "system.h"

#define DM_IMU_CAN_ID	0x01
#define DM_IMU_MST_ID	0x11

#define DM_IMU_ACCEL_REG        1   // 加速度计数据
#define DM_IMU_GYRO_REG         2   // 陀螺仪数据
#define DM_IMU_EULER_REG        3   // 欧拉角数据
#define DM_IMU_QUATERNION_REG   4   // 四元数数据

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

typedef struct
{
	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;

}IMU_t;

extern IMU_t imu;

void IMU_UpdateData(uint8_t* pData);
uint8_t IMU_RequestData(uint16_t can_id,uint8_t reg);

#endif