#include"UpperTest.h"

void Upper_Test_Task(void *argument){
    osDelay(100);
    for(;;){
        positionServo(800.0f,&hDJI[2]);
        // positionServo(-360.0f,&hDJI[3]);
        Distance_servo(700.0f,&hDJI[0]);
        CanTransmit_DJI_1234(&hcan1,0,0,hDJI[2].speedPID.output,0);
        // CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,-hDJI[0].speedPID.output,0,0);
        // CanTransmit_DJI_1234(&hcan1,0,0,0,hDJI[3].speedPID.output);
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


//KP=10最多500
//