#include"UpperTest.h"

void Upper_Test_Task(void *argument){
    osDelay(100);
    for(;;){
        positionServo(50.0f,&hDJI[2]);
        Distance_servo(1500.0f,&hDJI[0]);
        CanTransmit_DJI_1234(&hcan1,0,0,hDJI[2].speedPID.output,0);
        osDelay(1);
    }
}

void Upper_Test_Start(void)
{
    const osThreadAttr_t Upper_Test_attributes = {
        .name       = "Upper_Test",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    (void)osThreadNew(Upper_Test_Task, NULL, &Upper_Test_attributes);
}