#include"UpperStart.h"
#include "stdio.h"
void StartDefaultTask(void *argument){
    
    STP23L_Init(&huart1); // 开启距离传感接收中断
    DistanceUpdate_Start(); // 开启距离传感更新任务
    osDelay(1000);


    for(;;){
        printf("distance:%f\r\n",lidar.distance_aver);
        osDelay(50);
    }
}