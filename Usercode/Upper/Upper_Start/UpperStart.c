#include"UpperStart.h"

uint8_t pos[3] = {0,0,0};

void StartDefaultTask(void *argument){
    // while(1){osDelay(100);printf("111");}
    printf("Upper Start\r\n");
    Vision_Init(); // 初始化摄像头

    // Vision_Start(); // 启动摄像头
    // data_receive(pos);
    // Bean_Target_Set();
    Bean_Init(); 


    Angle_Init(); 
    STP23L_Init(&huart1); // 开启距离传感接收中断
    DistanceUpdate_Start(); // 开启距离传感更新任务
    
    osDelay(2500);//等待4s，方便烧录

    DJI_Init();//初始化电机参数

    // Bean_Target_Set();
    CANFilterInit(&hcan1); //初始化滤波器
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
        DebugPrint();
        // printf("%f %f %f\r\n", lidar.distance_aver, hDJI[0].FdbData.rpm, hDJI[1].FdbData.rpm);
        // printf("stage_flag:%d, lidar:%d, chassis:%f, claw:%f\r\n", stage_flag, (int)lidar.distance_aver, hDJI[2].AxisData.AxisAngle_inDegree, hDJI[3].AxisData.AxisAngle_inDegree);
        osDelay(50);
    }
}


