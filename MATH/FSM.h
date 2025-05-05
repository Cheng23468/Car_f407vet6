#ifndef __FSM_H
#define __FSM_H

typedef enum {
    CAR_EVENT_START = 0, // 启动事件
    CAR_EVENT_GET_CROSS, // 过交叉口事件
    CAR_EVENT_TURN_DONE, // 转向完成事件

    FSM_EVENT_MAX, // 最大事件
} FSM_Event_t; // 事件类型

typedef enum {
    CAR_STATE_TURN_LEFT = 0, // 左转
    CAR_STATE_TURN_RIGHT,        // 右转
    CAR_STATE_MOVE_FORWARD,     // 前进
    CAR_STATE_MOVE_BACKWARD,    // 后退
    CAR_STATE_STOP,            // 停止

    FSM_STATE_MAX,     // 最大状态
} FSM_State_t; // 状态类型

// 有限状态机结构体
typedef struct {
    FSM_State_t curState;        // 当前状态
    void (*state_func)(void); // 状态函数指针
    FSM_Event_t event;        // 事件
    FSM_State_t nextState;   // 下一个状态
} FSM_Table_t;

#endif