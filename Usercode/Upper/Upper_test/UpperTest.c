#include "UpperTest.h"
#include "Caculate.h"

typedef enum
{
    YAW_TEST_TO_POS_180 = 0,
    YAW_TEST_WAIT_POS_180,
    YAW_TEST_TO_ZERO,
    YAW_TEST_WAIT_ZERO
} YawTestState_t;

void Yaw_Test_180_Loop(void)
{
    static YawTestState_t yaw_test_state = YAW_TEST_TO_POS_180;
    static uint32_t wait_start_tick = 0U;

    switch (yaw_test_state) {
        case YAW_TEST_TO_POS_180:
            Yaw_servo(180.0f, &hDJI[2]);
            if (YawServo_IsArrived()) {
                wait_start_tick = HAL_GetTick();
                yaw_test_state = YAW_TEST_WAIT_POS_180;
            }
            break;

        case YAW_TEST_WAIT_POS_180:
            Yaw_servo(180.0f, &hDJI[2]);
            if ((HAL_GetTick() - wait_start_tick) >= 2000U) {
                YawServo_Reset();
                yaw_test_state = YAW_TEST_TO_ZERO;
            }
            break;

        case YAW_TEST_TO_ZERO:
            Yaw_servo(0.0f, &hDJI[2]);
            if (YawServo_IsArrived()) {
                wait_start_tick = HAL_GetTick();
                yaw_test_state = YAW_TEST_WAIT_ZERO;
            }
            break;

        case YAW_TEST_WAIT_ZERO:
            Yaw_servo(0.0f, &hDJI[2]);
            if ((HAL_GetTick() - wait_start_tick) >= 2000U) {
                YawServo_Reset();
                yaw_test_state = YAW_TEST_TO_POS_180;
            }
            break;

        default:
            YawServo_Reset();
            yaw_test_state = YAW_TEST_TO_POS_180;
            break;
    }
}

void Upper_Test_Task(void *argument)
{
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
