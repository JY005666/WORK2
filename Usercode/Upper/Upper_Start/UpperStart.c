#include"UpperStart.h"



void StartDefaultTask(void *argument){
    // while(1){osDelay(100);printf("111");}
    DJI_Init();//初始化电机参数
    CANFilterInit(&hcan1); //初始化滤波器
    STP23L_Init(&huart1); // 开启距离传感接收中断
    DistanceUpdate_Start(); // 开启距离传感更新任务
    Vision_Init(); // 初始化摄像头
    uint32_t start_time = HAL_GetTick();
    Vision_ClearAck();
    Vision_Start(0x11); // 识别数字
    if (!Vision_WaitAck(15000))
    {
        printf("vision ack timeout\r\n");
    }
    else
    {
        printf("vision ack ok\r\n");
    }  
    // Bean_Init(); //单独测试时用，记得删掉
    while((HAL_GetTick() - start_time)<=2500){osDelay(10);}//保证初始化时间足够
    Angle_Init(); 
    init_paramater(&par);
    //初始化夹爪两个舵机
    Claw_Init();
    
    //启动线程
    // Claw_test();
    // Upper_Test_Start();//上位机测试线程（串口）
    Upper_State_Start();
    Upper_Servo_Start();
    // Upper_Claw_Start();//好像没必要。。。

    for(;;){
        // DebugPrint();
        //  CanTransmit_DJI_1234(&hcan1,2500,-2500,hDJI[2].speedPID.output,hDJI[3].speedPID.output);   
        printf("%f %f %f\r\n", lidar.distance_aver, hDJI[0].FdbData.rpm, hDJI[1].FdbData.rpm);
        // printf("stage_flag:%d, lidar:%d, chassis:%f, claw:%f\r\n", stage_flag, (int)lidar.distance_aver, hDJI[2].AxisData.AxisAngle_inDegree, hDJI[3].AxisData.AxisAngle_inDegree);
        osDelay(50);
    }
}


