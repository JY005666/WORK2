#include"UpperClaw.h"

void Upper_Claw_Task(void *argument){
    osDelay(100);
    for(;;){
        
        osDelay(10);
    }
}


void Upper_Claw_Start(void)
{
    const osThreadAttr_t Upper_Claw_attributes = {
        .name       = "Upper_Claw",
        .stack_size = 128 * 5,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    (void)osThreadNew(Upper_Claw_Task, NULL, &Upper_Claw_attributes);
}
void Claw_Init(uint8_t servoID){
    printf("Initializing servo %d...\r\n", servoID);
    // 1. Test communication
    int pingID = Ping(servoID);
    if (pingID == -1) {
        printf("Error: Failed to connect to servo %d. Please check the cable, power or ID.\r\n", servoID);
        printf("错误: 无法连接到舵机 %d，请检查线路、电源或 ID。\r\n", servoID);
        return;
    }
    printf("成功: 舵机 %d 在线！\r\n", pingID);

    // 2. 初始化设置
    unLockEprom(servoID);   // 解锁参数修改权限
    EnableTorque(servoID, 1); // 开启扭矩，舵机进入锁定状态
    printf("舵机已使能扭矩。\r\n");
}
void Claw_test(){
    // WritePosEx(1,1200,1000,100);
    // WritePosEx(2,1900,1000,100);
    // osDelay(1500);
    WritePosEx(1,2700,1000,100);
    osDelay(1500);
    WritePosEx(1,1700,1000,100);
}

//夹爪角度1100，750，1500，