#include "DistanceUpdate.h"
#include "UpperStart.h"


void UartUpdateTask(void *argument)
{

    for (;;) {
        
            if (UartFlag[0]) {
                STP_23L_Decode(Rxbuffer_1, &lidar);
                UartFlag[0] = 0;
            }
            osDelay(1);
        
    }
    /* USER CODE END UartUpdateTask */
}

void DistanceUpdate_Start()
{
    const osThreadAttr_t DistanceUpdate_attributes = {
        .name       = "DistanceUpdate",
        .stack_size = 128 * 5,
        .priority   = (osPriority_t)osPriorityAboveNormal,
    };
    (void)osThreadNew(UartUpdateTask, NULL, &DistanceUpdate_attributes);
}
