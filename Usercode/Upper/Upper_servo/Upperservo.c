#include "Upperservo.h"

void Upper_Servo_Task(void *argument)
{
    osDelay(100);
    for (;;) {
        Distance_servo(par.target_distance,&hDJI[0]);
        positionServo(par.degree_chassis,&hDJI[2]);
        // positionServo(80.0,&hDJI[2]);
        // Distance_servo(2000.0,&hDJI[0]);
        CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,-hDJI[0].speedPID.output,hDJI[2].speedPID.output,par.torque_offset);
        // CanTransmit_DJI_1234(&hcan1,-700,700,hDJI[2].speedPID.output,-1300);
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

void Servo_test(void){
    for(;;){
        positionServo(-510.0f,&hDJI[2]);
        Distance_servo(1500,&hDJI[0]);
        CanTransmit_DJI_1234(&hcan1,0,0,hDJI[2].speedPID.output,0);
        osDelay(1);
    }

}