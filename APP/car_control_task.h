#ifndef __CAR_CONTROL_TASK_H
#define __CAR_CONTROL_TASK_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"
#include "TD.h"
#include "FSM.h"

#define CAR_CONTROL_TASK_PRIO		4     //Task priority //任务优先级
#define CAR_CONTROL_STK_SIZE 		256   //Task stack size //任务堆栈大小

typedef enum {
    CAR_CONTROL_USE_SENSOR = 0, // 使用传感器循迹
    CAR_CONTROL_USE_ODOMETRY,   // 使用里程计循迹
} CarControlFlag_t;

typedef struct {
    CarControlFlag_t control_flag; // 控制方式
} CarControl_t;

extern CarControl_t CarControl; // 小车控制结构体

void car_control_task(void *pvParameters);

#endif
