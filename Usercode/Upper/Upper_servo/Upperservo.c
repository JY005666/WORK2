#include "Upperservo.h"

void Upper_Servo_Task(void *argument)
{
    osDelay(100);
    for (;;) {
        Distance_servo(1000.0f,&hDJI[0]);
        CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,-hDJI[0].speedPID.output,0,-1300);
        osDelay(2);
    }
    
}

void Upper_Servo_Start(void)
{
    osThreadId_t Upper_ServoHandle;
    const osThreadAttr_t Upper_Servo_attributes = {
        .name       = "Upper_Servo",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    Upper_ServoHandle = osThreadNew(Upper_Servo_Task, NULL, &Upper_Servo_attributes);
}