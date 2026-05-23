#include"UpperStart.h"
void StartDefaultTask(void *argument){
    // while(1){osDelay(100);}
    STP23L_Init(&huart1); // 开启距离传感接收中断
    DistanceUpdate_Start(); // 开启距离传感更新任务
    
    osDelay(4000);//等待4s，方便烧录

    DJI_Init();//初始化电机参数
    CANFilterInit(&hcan1);  //初始化滤波器
    init_paramater(&par);
    //初始化夹爪两个舵机
    Claw_Init(1);
    Claw_Init(2);

    //启动线程
    // Claw_test();
    Upper_Test_Start();
    // Upper_State_Start();
    // Upper_Servo_Start();
    // Upper_Claw_Start();//好像没必要。。。

    for(;;){
        printf("distance:%f,%f,%f,%f\r\n",lidar.distance_aver,hDJI[0].FdbData.rpm,hDJI[2].AxisData.AxisAngle_inDegree,hDJI[3].AxisData.AxisAngle_inDegree);
        osDelay(50);
    }
}