#include "UpperState.h"
#include"stdio.h"
#include"stdlib.h"
paramater par;

uint16_t stage_flag = 0;

Bean bean[3];
Box box[5];

void Bean_Init(void){
    bean[0].position = RIGHT;
    bean[1].position = LEFT;
    bean[2].position = MIDDLE;
    
    //视觉识别结果

} 
void Box_Init(void){
    box[0].position = LEFT_2;
    box[1].position = LEFT_1;
    box[2].position = MIDDLE_0;
    box[3].position = RIGHT_1;
    box[4].position = RIGHT_2;

    //视觉识别
    bean[2].target_position = MIDDLE_0;
    bean[1].target_position = RIGHT_1;
    bean[0].target_position = LEFT_1;

}
void Bean_Target_Set(void){
    for(int i=0;i<3;i++){
        if(bean[i].color == YELLOW) bean[i].target_number = 1;
        else if (bean[i].color == GREEN) bean[i].target_number = 2;
        else if (bean[i].color == WHITE ) bean[i].target_number = 3;
        for(int j=0;j<5;j++){
            if(bean[i].target_number == box[j].number) bean[i].target_position = box[j].position;
        }
    }
}
void init_paramater(paramater *par){
    par->target_distance = lidar.distance_aver;
    par->degree_chassis = 0.0;
    par->torque_offset = 0;
}

void Upper_State_Task(void *arg){
    for(;;){
        if(stage_flag == 0){//到达中间豆子抓取位置，张开爪子
            par.torque_offset = -2000;
            par.degree_chassis = 60.0f;
            par.target_distance = 575.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -540.0f){
                par.torque_offset = -1300;
                if(lidar.distance_aver < 700.0){
                    par.degree_chassis = 0.0f;
                    if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree)<1.0f)){
                        WritePosEx(1,1850,1000,100);
                        WritePosEx(2,1100,1000,100);
                        stage_flag = 10;
                    }
                }
            }
        }
        if(stage_flag == 10){//向下抓取
            osDelay(300);
            par.torque_offset = 800;
            osDelay(800);
            // WritePosEx(1,800,1000,100);
            osDelay(800);
            stage_flag = 20;
        }
        if(stage_flag == 20){//抬升并旋转
            par.torque_offset = -2000;
            pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
            if(hDJI[3].AxisData.AxisAngle_inDegree < -580.0f){
                par.torque_offset = -2000;
                // Motor_State_Reset(&hDJI[2]);
                par.degree_chassis = -630.0f;
                if(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f){
                    stage_flag = 30;
                    Motor_State_Reset(&hDJI[0]);
                    Motor_State_Reset(&hDJI[2]);
                }
            }
        }
        if(stage_flag == 30){
            switch(bean[2].target_position){
                case LEFT_2: {
                }break;
                case LEFT_1: {
                }break;
                case MIDDLE: {
                    WritePosEx(2,1100,1000,100);
                    par.target_distance = 2275.0f;
                    pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                    if(lidar.distance_aver>1800.0f){
                        par.degree_chassis = -550.0f;
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.torque_offset = 1000;
                        }
                        if(hDJI[3].AxisData.AxisAngle_inDegree  >-400.0f){
                            WritePosEx(1,1300,1000,100);
                            Motor_State_Reset(&hDJI[0]);
                            Motor_State_Reset(&hDJI[2]);
                            stage_flag = 40;
                        }
                    }
                } break;
                case RIGHT_1: {
                }break;
                case RIGHT_2: {
                }break;
            }
        }
        if(stage_flag == 40){ //抓取左边豆子
            par.torque_offset = -2900;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -580.0f){
                par.torque_offset = -1300;
                
                par.degree_chassis = -100.0f;
                pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                par.target_distance = 159.0f;
                if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)){
                    WritePosEx(1,2700,1000,100);
                    WritePosEx(2,750,1000,100);
                    stage_flag = 50;
                }
            }
        }
        if(stage_flag == 50){//抓取
            osDelay(300);
            par.torque_offset = 1700;
            osDelay(800);
            // WritePosEx(1,800,1000,100);
            osDelay(800);
            stage_flag = 60;
        }
        if(stage_flag == 60){
            par.torque_offset = -3600;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -600.0f){
                par.torque_offset = -3000;
                // Motor_State_Reset(&hDJI[2]);
                
                par.degree_chassis = -630.0f;
                pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                if(hDJI[2].AxisData.AxisAngle_inDegree<-600.0f){
                    stage_flag = 70;
                    Motor_State_Reset(&hDJI[0]);
                    Motor_State_Reset(&hDJI[2]);
                }
            }
        }
        if(stage_flag == 70){
            switch(bean[1].target_position){
                case LEFT_2: {
                }break;
                case LEFT_1: {
                }break;
                case MIDDLE: {
                } break;
                case RIGHT_1: {
                    WritePosEx(1,1400,1600,100);
                    par.target_distance = 2355.0f;
                    if(lidar.distance_aver>1800.0f){
                        par.degree_chassis = -463.0f;
                        pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.torque_offset = 500;
                        }
                        if(hDJI[3].AxisData.AxisAngle_inDegree  >-250.0f){
                            WritePosEx(1,1300,1600,100);
                            stage_flag = 80;
                        }
                    } 
                }break;
                case RIGHT_2: {
                }break;
            }
        }
        if(stage_flag == 80){ //抓取右边豆子
            par.torque_offset = -2500;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -600.0f){
                par.torque_offset = -1300;
                
                par.degree_chassis = -975.0f;
                pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                par.target_distance = 169.0f;
                if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)){
                    WritePosEx(1,1850,1000,100);
                    WritePosEx(2,1500,1000,100);
                    stage_flag = 90;
                }
            }
        }
        if(stage_flag == 90){//向下抓取
            osDelay(300);
            par.torque_offset = 800;
            osDelay(800);
            WritePosEx(1,800,1000,100);
            osDelay(800);
            stage_flag = 100;
        }
        if(stage_flag == 100){//抬升并旋转
            par.torque_offset = -3400;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -600.0f){
                par.torque_offset = -3000;
                // Motor_State_Reset(&hDJI[2]);
                
                par.degree_chassis = -1700.0f;
                pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                if(hDJI[2].AxisData.AxisAngle_inDegree<-1600.0f){
                    stage_flag = 110;
                    Motor_State_Reset(&hDJI[0]);
                }
            }
        }
        if(stage_flag == 110){
            switch(bean[0].target_position){
                case LEFT_2: {
                }break;
                case LEFT_1: {
                    WritePosEx(2,800,1000,100);
                    par.target_distance = 2350.0f;
                    if(lidar.distance_aver>1800.0f){
                        
                        par.degree_chassis = -1691.0f;
                        pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.torque_offset = 500;
                        }
                        if(hDJI[3].AxisData.AxisAngle_inDegree  >-250.0f){
                            WritePosEx(1,1300,1000,100);
                            stage_flag = 1000;
                        }
                    }
                }break;
                case MIDDLE: {
                } break;
                case RIGHT_1: {
                }break;
                case RIGHT_2: {
                }break;
            }
        }

        osDelay(2);
    }
}

void Upper_State_Start(void)
{
    const osThreadAttr_t Upper_State_attributes = {
        .name       = "Upper_State",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    (void)osThreadNew(Upper_State_Task, NULL, &Upper_State_attributes);
}