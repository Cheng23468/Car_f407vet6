#include "car_control_task.h"

CarControl_t CarControl; // 小车控制结构体
float target_theta = 0;

void car_control_task(void *pvParameters)
{
    vTaskDelay(11000);

    uint32_t lastWakeTime = getSysTickCnt();
    for (;;)
    {
        /* 此任务以100Hz的频率运行(10ms控制一次) */ 
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

        /* Loop */
        // target_theta = PI/2;
        vTaskDelay(2000);
        target_theta = 0;
        vTaskDelay(2000);
    }
}