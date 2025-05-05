#include "motor_task.h"

unsigned char rx_buff[256] = {0};

/* Debug Variables */
uint16_t motor_pwm_set = 0;
extern float target_theta;

// 循迹进程
void motor_task(void *pvParameters)
{
    /* Setup */
    
    TrackingSensor_init(&GrayTrackingSensor, 80, 0.0f);               // 初始化灰度循迹传感器
    PID_Init(&GrayTrackingSensor.PID, 0.3f, 0.0f, 0.0f, -1.0f, 5.0f); // 初始化位置循迹PID控制器
    PID_Init(&Car_Theta_PID, 18.0f, 0.0f, 0.0f, -1.0f, 8.0f); // 初始化小车航向角PID控制器
    TD_Init(&Car_Theta_TD, 10.0f, 0.01, -1.0); // 初始化小车航向角微分跟踪器
    vTaskDelay(1000);   //延时1秒

    /* 循迹模块初始化 Begin */
    while (Ping())
    {
        vTaskDelay(1);
        sprintf((char *)rx_buff, "Ping Faild Try Again!\r\n");
        UART_Transmit(UART5, rx_buff, (uint16_t)strlen((char *)rx_buff));
        memset(rx_buff, 0, 256);
    }
    sprintf((char *)rx_buff, "Ping Succseful!\r\n");
    UART_Transmit(UART5, rx_buff, (uint16_t)strlen((char *)rx_buff));
    memset(rx_buff, 0, 256);
    /* 循迹模块初始化 End */
   
    uint32_t lastWakeTime = getSysTickCnt();
    for (;;)
    {
        /* 此任务以100Hz的频率运行(10ms控制一次) */ 
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

        /* Loop */
        /* 更新传感器数据 */
        TrackingSensor_update(&GrayTrackingSensor);
        if(CarControl.control_flag == CAR_CONTROL_USE_SENSOR) {
            PID_Generate(&GrayTrackingSensor.PID, GrayTrackingSensor.position, GrayTrackingSensor.target_position);
            Move_Z = PID_GetOutput(&GrayTrackingSensor.PID);
        }
        else if(CarControl.control_flag == CAR_CONTROL_USE_ODOMETRY) {
            TD_Update(&Car_Theta_TD, target_theta); // 更新小车航向角微分跟踪器
            PID_Generate(&Car_Theta_PID, Diff_Car_Odometry.Odometry.theta, TD_GetValue(&Car_Theta_TD)); // 更新小车航向角PID控制器
            Move_Z = TD_GetDerivative(&Car_Theta_TD) + PID_GetOutput(&Car_Theta_PID); // 获取小车航向角PID控制器的输出值
        }        

        /* Debug Code */
        // sprintf((char *)rx_buff, "%f,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", GrayTrackingSensor.position,GrayTrackingSensor.state ,GrayTrackingSensor.sensor_values[0],GrayTrackingSensor.sensor_values[1],GrayTrackingSensor.sensor_values[2],GrayTrackingSensor.sensor_values[3],GrayTrackingSensor.sensor_values[4],GrayTrackingSensor.sensor_values[5],GrayTrackingSensor.sensor_values[6],GrayTrackingSensor.sensor_values[7]);
        sprintf((char *)rx_buff, "%f, %f, %f, %f\n", target_theta, TD_GetValue(&Car_Theta_TD), Diff_Car_Odometry.Odometry.theta, Move_Z);
        UART_Transmit(UART5, rx_buff, (uint16_t)strlen((char *)rx_buff));
        memset(rx_buff, 0, 256);
        // vTaskDelay(1500);
        // Move_X = 0.8;
        // // motor_pwm_set = 2000;
        // vTaskDelay(1000);
        // // motor_pwm_set = 3000;
        // vTaskDelay(1000);
        // Move_X = 0.5;
        // vTaskDelay(1000);
        // Move_X = -0.5;
        // // motor_pwm_set = 0;
        // vTaskDelay(2000);
    }
}