#include "car_control_task.h"

#define FORWARD_SPEED       0.50f
#define FORWARD_SPEED_LOW   0.3f
#define FORWARD_SPEED_LOW2  0.4f
#define TURN_SPEED          0.0f
#define BACKWARD_SPEED      -0.3f

#define TURN_ANGLE_THRESHOLD  0.1f  // 角度转弯判断阈值
#define FORWARD_METER_THRESHOLD  0.005f // 前进距离判断阈值
#define FORWARD_SLOW_DOWN_THRESHOLD 0.13    // 提前减速的距离阈值

#define CROSS_1_X       0.50f
#define CROSS_2_Y       0.22f
#define CROSS_3_X       1.30f
#define CROSS_4_Y       1.01f
#define CROSS_5_X       0.55f
#define CROSS_6_Y       1.3f

#define PARKING_X_1    0.78f    // 第一行停车位的X坐标(从下往上数)
#define PARKING_X_2    0.10f    // 第二行停车位的X坐标
#define PARKING_Y_1    -0.05f    // 第一列停车位的Y坐标(从右往左数)
#define PARKING_Y_2    0.53f    // 第二列停车位的Y坐标
#define PARKING_Y_3    0.50f    // 第三列停车位的Y坐标
#define PARKING_Y_4    1.30f    // 第四列停车位的Y坐标


#define PARKING_SPOT_1_X  PARKING_X_1
#define PARKING_SPOT_2_X  PARKING_X_1
#define PARKING_SPOT_5_X  PARKING_X_1
#define PARKING_SPOT_6_X  PARKING_X_1

#define PARKING_SPOT_3_X  PARKING_X_2
#define PARKING_SPOT_4_X  PARKING_X_2
#define PARKING_SPOT_7_X  PARKING_X_2
#define PARKING_SPOT_8_X  PARKING_X_2

#define PARKING_SPOT_2_Y  PARKING_Y_1
#define PARKING_SPOT_4_Y  PARKING_Y_1

#define PARKING_SPOT_1_Y  PARKING_Y_2
#define PARKING_SPOT_3_Y  PARKING_Y_2

#define PARKING_SPOT_6_Y  PARKING_Y_3
#define PARKING_SPOT_8_Y  PARKING_Y_3

#define PARKING_SPOT_5_Y  PARKING_Y_4
#define PARKING_SPOT_7_Y  PARKING_Y_4


typedef enum {
    CAR_CROSS_JUDGE_X,
    CAR_CROSS_JUDGE_Y,
} CarCrossJudge_t;

CarControl_t CarControl; // 小车控制结构体
float target_theta = 0;

uint8_t start_flag=0;
CarTask_t car_task_flag = CAR_TASK_1;
CarParkingSpot_t car_parking_spot_flag = CAR_PARKING_NONE;

void car_forward_until_cross(float forward_speed, CarCrossJudge_t Judge, float cross_value) {
    Move_X = forward_speed;
    if(Judge == CAR_CROSS_JUDGE_X) {
        while(fabs(Diff_Car_Odometry.Odometry.x - cross_value) > FORWARD_METER_THRESHOLD) {   
            vTaskDelay(2);
            if(fabs(Diff_Car_Odometry.Odometry.x - cross_value) < FORWARD_SLOW_DOWN_THRESHOLD) {
                Move_X = 0.08f;
                Buzzer=1;
                vTaskDelay(1);
                Buzzer=0;
            }
        }
    }
    else if(Judge == CAR_CROSS_JUDGE_Y) {
        while(fabs(Diff_Car_Odometry.Odometry.y - cross_value) > FORWARD_METER_THRESHOLD) {   
            vTaskDelay(2);
            if(fabs(Diff_Car_Odometry.Odometry.y - cross_value) < FORWARD_SLOW_DOWN_THRESHOLD) {
                Move_X = 0.08f;
                Buzzer=1;
                vTaskDelay(1);
                Buzzer=0;
            }
        }
    }
}

void car_cross_turn(float turn_speed, float turn_angle) {
    Move_X = turn_speed;
    target_theta = turn_angle;
    CarControl.control_flag = CAR_CONTROL_USE_ODOMETRY;
    while(fabsf(target_theta - Diff_Car_Odometry.Odometry.theta) > TURN_ANGLE_THRESHOLD) {
        vTaskDelay(10);
    }
}

void car_forward_until_stop(float forward_speed, CarCrossJudge_t Judge, float stop_value) {
    Move_X = forward_speed;
    if(Judge == CAR_CROSS_JUDGE_X) {
        while(fabs(Diff_Car_Odometry.Odometry.x - stop_value) > FORWARD_METER_THRESHOLD) {   
            vTaskDelay(10);
        }
    }
    else if(Judge == CAR_CROSS_JUDGE_Y) {
        while(fabs(Diff_Car_Odometry.Odometry.y - stop_value) > FORWARD_METER_THRESHOLD) {   
            vTaskDelay(10);
        }
    }
    Move_X = 0;
}

void car_control_task(void *pvParameters)
{
    CarControl.control_flag = CAR_CONTROL_USE_ODOMETRY;
    CarControl.parking_spot = CAR_PARKING_NONE;

    vTaskDelay(1000);

    uint32_t lastWakeTime = getSysTickCnt();
    for (;;)
    {
        /* 此任务以100Hz的频率运行(10ms控制一次) */ 
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
        // Debug_Printf("car_control_task\r\n");
        /* Loop */
        uint8_t key_status = click_N_Double(30);
        // if(1 == key_status) {
        //     start_flag = 0;
        //     car_task_flag ++;
        //     if(car_task_flag > CAR_TASK_3) {
        //         car_task_flag = CAR_TASK_1;
        //     }
        if(1 == key_status) {
            start_flag = 0;
            CarControl.parking_spot ++;
            if(CarControl.parking_spot >= CAR_PARKING_SPOT_MAX) {
                CarControl.parking_spot = CAR_PARKING_NONE;
            }
        }
        if(2 == key_status) {
            // Debug_Printf("按键按下\r\n");
            Led_Count = 500;
            vTaskDelay(500);
            Buzzer = 1;
            vTaskDelay(50);
            Buzzer = 0;
            vTaskDelay(50);
            Buzzer = 1;
            vTaskDelay(50);
            Buzzer = 0;
            vTaskDelay(50);
            Buzzer = 1;
            vTaskDelay(50);
            Buzzer = 0;
            start_flag = 1;
        }

        if(start_flag) {
            // 开始前进 
            // 第一个路口
            car_forward_until_cross(FORWARD_SPEED, CAR_CROSS_JUDGE_X, CROSS_1_X);
            car_cross_turn(TURN_SPEED, PI/2);
            // Debug_Printf("第一个路口\r\n");

            // 第二个路口
            car_forward_until_cross(FORWARD_SPEED_LOW2, CAR_CROSS_JUDGE_Y, CROSS_2_Y);
            car_cross_turn(TURN_SPEED, 0);
            // Debug_Printf("第二个路口\r\n");

            if(CarControl.parking_spot == CAR_PARKING_SPOT_1) {
                // Debug_Printf("停车位1\r\n");
                car_forward_until_cross(FORWARD_SPEED_LOW2, CAR_CROSS_JUDGE_X, PARKING_SPOT_1_X);
                Move_X = 0.0f;
                car_cross_turn(TURN_SPEED, PI/2);
                car_forward_until_stop(FORWARD_SPEED_LOW, CAR_CROSS_JUDGE_Y, PARKING_SPOT_1_Y);
                vTaskDelay(1000);
                car_forward_until_stop(BACKWARD_SPEED, CAR_CROSS_JUDGE_Y, CROSS_2_Y+0.10f);
                car_cross_turn(TURN_SPEED, 0);
                // Debug_Printf("继续行驶\r\n");
            }
            else if(CarControl.parking_spot == CAR_PARKING_SPOT_2) {
                // Debug_Printf("停车位2\r\n");
                car_forward_until_cross(FORWARD_SPEED, CAR_CROSS_JUDGE_X, PARKING_SPOT_2_X);
                Move_X = 0.0f;
                car_cross_turn(TURN_SPEED, -PI/2);
                car_forward_until_stop(FORWARD_SPEED_LOW, CAR_CROSS_JUDGE_Y, PARKING_SPOT_2_Y);
                vTaskDelay(1000);
                car_forward_until_stop(BACKWARD_SPEED, CAR_CROSS_JUDGE_Y, CROSS_2_Y-0.10f);
                car_cross_turn(TURN_SPEED, 0);
                // Debug_Printf("继续行驶\r\n");
            }
            else if(CarControl.parking_spot == CAR_PARKING_SPOT_3) {
                // Debug_Printf("停车位3\r\n");
                car_forward_until_cross(FORWARD_SPEED, CAR_CROSS_JUDGE_X, PARKING_SPOT_3_X);
                Move_X = 0.0f;
                car_cross_turn(TURN_SPEED, PI/2);
                car_forward_until_stop(FORWARD_SPEED_LOW, CAR_CROSS_JUDGE_Y, PARKING_SPOT_3_Y);
                vTaskDelay(1000);
                car_forward_until_stop(BACKWARD_SPEED, CAR_CROSS_JUDGE_Y, CROSS_2_Y+0.10f);
                car_cross_turn(TURN_SPEED, 0);
                // Debug_Printf("继续行驶\r\n");
            }
            else if(CarControl.parking_spot == CAR_PARKING_SPOT_4) {
                // Debug_Printf("停车位4\r\n");
                car_forward_until_cross(FORWARD_SPEED, CAR_CROSS_JUDGE_X, PARKING_SPOT_4_X);
                Move_X = 0.0f;
                car_cross_turn(TURN_SPEED, -PI/2);
                car_forward_until_stop(FORWARD_SPEED_LOW, CAR_CROSS_JUDGE_Y, PARKING_SPOT_4_Y);
                vTaskDelay(1000);
                car_forward_until_stop(BACKWARD_SPEED, CAR_CROSS_JUDGE_Y, CROSS_2_Y-0.10f);
                car_cross_turn(TURN_SPEED, 0);
                // Debug_Printf("继续行驶\r\n");
            }
            // else if(CarControl.parking_spot == CAR_PARKING_SPOT_2) {
            //     car_forward_until_stop(FORWARD_SPEED, CAR_CROSS_JUDGE_Y, 0);
            //     Debug_Printf("停车位2\r\n");
            // }
            // else if(CarControl.parking_spot == CAR_PARKING_SPOT_3) {
            //     car_forward_until_stop(FORWARD_SPEED, CAR_CROSS_JUDGE_X, 0);
            //     Debug_Printf("停车位3\r\n");
            // }
            // else if(CarControl.parking_spot == CAR_PARKING_SPOT_4) {
            //     car_forward_until_stop(FORWARD_SPEED, CAR_CROSS_JUDGE_Y, 0);
            //     Debug_Printf("停车位4\r\n");
            // }

            // 第三个路口
            car_forward_until_cross(FORWARD_SPEED, CAR_CROSS_JUDGE_X, CROSS_3_X);
            car_cross_turn(TURN_SPEED, PI/2);
            // Debug_Printf("第三个路口\r\n");

            // 第四个路口
            car_forward_until_cross(FORWARD_SPEED, CAR_CROSS_JUDGE_Y, CROSS_4_Y);
            car_cross_turn(TURN_SPEED, PI);
            // Debug_Printf("第四个路口\r\n");

            // 第五个路口
            car_forward_until_cross(FORWARD_SPEED, CAR_CROSS_JUDGE_X, CROSS_5_X);
            car_cross_turn(TURN_SPEED, PI/2);
            // Debug_Printf("第五个路口\r\n");

            // 第六个路口
            car_forward_until_cross(FORWARD_SPEED_LOW2, CAR_CROSS_JUDGE_Y, CROSS_6_Y);
            car_cross_turn(TURN_SPEED, PI);
            // Debug_Printf("第六个路口\r\n");

            // 停止
            car_forward_until_cross(FORWARD_SPEED, CAR_CROSS_JUDGE_X, 0);
            Move_X = 0;
            // Debug_Printf("停止\r\n");

            Buzzer = 1;
            vTaskDelay(50);
            Buzzer = 0;
            vTaskDelay(50);
            Buzzer = 1;
            vTaskDelay(50);
            Buzzer = 0;
            vTaskDelay(50);
            Buzzer = 1;
            vTaskDelay(50);
            Buzzer = 0;

            start_flag = 0;
        }

        // if(1 == click_N_Double(30)) {
        //     Led_Count = 100;
        //     Buzzer = 1;
        //     vTaskDelay(200);
        //     Buzzer = 0;
        //     start_flag = 1;
        // }

        // if(start_flag) {
        //     CarControl.control_flag = CAR_CONTROL_USE_SENSOR;
        //     Move_X = 0.2f;

        //     if(GrayTrackingSensor.state == TrackingSensor_AllBlack) {
        //         Move_X = 0;
        //         start_flag = 0;
        //     }
        // }
        
    }
}