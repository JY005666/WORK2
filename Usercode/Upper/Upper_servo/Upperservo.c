#include "Upperservo.h"

void Upper_Servo_Task(void *argument)
{
    osDelay(100);
    for (;;) {
        Distance_servo(par.target_distance,&hDJI[0]);
        positionServo(par.degree_chassis,&hDJI[2]);
        positionServo(par.degree_claw,&hDJI[3]);
        CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,-hDJI[0].speedPID.output,hDJI[2].speedPID.output,hDJI[3].speedPID.output);        
        osDelay(1);
    }
}

void Upper_Servo_Start(void)
{
    const osThreadAttr_t Upper_Servo_attributes = {
        .name       = "Upper_Servo",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    (void)osThreadNew(Upper_Servo_Task, NULL, &Upper_Servo_attributes);
}
