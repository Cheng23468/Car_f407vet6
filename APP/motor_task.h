#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"
#include "gw_gray_software_iic.h"
#include "tracking_sensor.h"
#include "PID.h"
#include "TD.h"
#include "Odometry.h"
#include "car_control_task.h"

#define MOTOR_TASK_PRIO		4     //Task priority //任务优先级
#define MOTOR_STK_SIZE 		512   //Task stack size //任务堆栈大小

void motor_task(void *pvParameters);

#endif
