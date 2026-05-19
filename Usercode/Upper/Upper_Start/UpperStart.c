#include"UpperStart.h"
void StartDefaultTask(void *argument){
    
    STP23L_Init(&huart1); // 开启距离传感接收中断
    DistanceUpdate_Start(); // 开启距离传感更新任务
    
    osDelay(4000);//等待4s，方便烧录
    DJI_Init();//初始化电机参数
    CANFilterInit(&hcan1);

    //启动线程
    Upper_Servo_Start();

    for(;;){
        printf("distance:%f,%f,%f\r\n",lidar.distance_aver,hDJI[0].speedPID.output,hDJI[0].posPID.output);
        osDelay(50);
    }
}