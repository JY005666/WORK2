#include "UpperTest.h"
#include "Caculate.h"

void Upper_Test_Task(void *argument){
    osDelay(100);
    HostControl_Start(&huart4);
    for(;;){
        int16_t iq0 = 0;
        int16_t iq2 = 0;
        int16_t iq3 = 0;

        HostControl_Process();

        if (g_host_control.stop_all) {
            CanTransmit_DJI_1234(&hcan1, 0, 0, 0, 0);
            osDelay(1);
            continue;
        }

        if (g_host_control.enabled[0]) {
            Distance_servo(g_host_control.target_deg[0], &hDJI[0]);
            iq0 = (int16_t)hDJI[0].speedPID.output;
        }

        if (g_host_control.enabled[2]) {
            Yaw_servo(g_host_control.target_deg[2], &hDJI[2]);
            iq2 = (int16_t)hDJI[2].speedPID.output;
        }

        if (g_host_control.enabled[3]) {
            positionServo(g_host_control.target_deg[3], &hDJI[3]);
            iq3 = (int16_t)hDJI[3].speedPID.output;
        }

        CanTransmit_DJI_1234(&hcan1, iq0, -iq0, iq2, iq3);
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
