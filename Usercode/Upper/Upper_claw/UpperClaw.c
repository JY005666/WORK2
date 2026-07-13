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
void Claw_Init(void){ //开启时钟和PWM接收
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
void Claw_degree_set(uint8_t degree, uint8_t servoID){
    if(servoID == 1){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (int)degree*1000/90+500);
    }
    else if(servoID == 2){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (int)degree*1000/90+500);
    }
}
void Claw_test(){
    Claw_degree_set(120, 2);
    osDelay(1500);
    Claw_degree_set(50, 2);


}

//夹取2600 张开（中位校准） 1900闭合
//旋转 中间2048 左边豆子 1725 右边豆子 2450 R1 2350 L1 1800