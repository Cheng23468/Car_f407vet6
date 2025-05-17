#ifndef __IMU_TASK_H
#define __IMU_TASK_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"


#define IMU_TASK_PRIO		7     //Task priority //任务优先级
#define IMU_STK_SIZE 		512   //Task stack size //任务堆栈大小

void imu_task(void *pvParameters);

#endif
