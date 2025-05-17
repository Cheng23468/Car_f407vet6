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

typedef enum {
    CAR_TASK_1 = 1,
    CAR_TASK_2,
    CAR_TASK_3,
} CarTask_t;

typedef enum {
    CAR_PARKING_NONE = 0,   // 不停车
    CAR_PARKING_SPOT_1,     // 停车位1
    CAR_PARKING_SPOT_2,     // 停车位2
    CAR_PARKING_SPOT_3,     // 停车位3
    CAR_PARKING_SPOT_4,     // 停车位4
    CAR_PARKING_SPOT_5,     // 停车位5
    CAR_PARKING_SPOT_6,     // 停车位6
    CAR_PARKING_SPOT_7,     // 停车位7
    CAR_PARKING_SPOT_8,     // 停车位8

    CAR_PARKING_SPOT_MAX, // 停车位最大值
} CarParkingSpot_t;

typedef struct {
    CarControlFlag_t control_flag; // 控制方式
    CarParkingSpot_t parking_spot; // 要停的停车位
} CarControl_t;

extern CarControl_t CarControl; // 小车控制结构体
extern CarTask_t car_task_flag;

void car_control_task(void *pvParameters);

#endif
