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
    bean[2].target_position = LEFT_2;
    bean[1].target_position = RIGHT_1;
    bean[0].target_position = RIGHT_2;

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
    par->degree_claw = 0.0;
}

void Upper_State_Task(void *arg){
    for(;;){
        if(stage_flag == 0){//到达中间豆子抓取位置，张开爪子
            pid_reset(&hDJI[3], 5.0f, 0.0f, 0.0f);
            par.degree_claw = -600.0f;
            par.degree_chassis = 60.0f;
            par.target_distance = 590.0f;
            if(lidar.distance_aver < 700.0){
                par.degree_chassis = 4.0f;
                if((abs(lidar.distance_aver-par.target_distance)<8.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<0.5f)){
                    WritePosEx(1,2800,1000,100);
                    WritePosEx(2,2048,1000,100);
                    stage_flag = 10;
                }
            }
        }
        if(stage_flag == 10){//向下抓取
            osDelay(300);
            par.degree_claw = -250.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree>-450.0f){
                WritePosEx(1,1900,1000,100);
                osDelay(1000);
                Motor_State_Reset(&hDJI[2]);
                Motor_State_Reset(&hDJI[0]);
                stage_flag = 20;
            }
        }
        if(stage_flag == 20){//抬升并旋转
            pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
            pid_reset(&hDJI[3], 10.0f, 0.0f, 0.0f);
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -550.0f){
                // Motor_State_Reset(&hDJI[2]);
                par.degree_chassis = -630.0f;
                if(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<10.0f){
                    stage_flag = 30;
                    Motor_State_Reset(&hDJI[0]);
                    Motor_State_Reset(&hDJI[2]);
                }
            }
        }
        if(stage_flag == 30){
            switch(bean[2].target_position){
                case LEFT_2: {
                    WritePosEx(2,2048,1000,100);
                    par.target_distance = 2655.0f;
                    pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                    par.degree_chassis = -755.0f;
                    if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                        par.degree_claw = -170.0f;
                        if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-230.0f){
                            WritePosEx(1,2300,1000,100);
                            osDelay(500);
                            Motor_State_Reset(&hDJI[0]);
                            Motor_State_Reset(&hDJI[2]);
                            stage_flag = 31;//有点极限，退后一点再转向
                        }
                        }
                }break;
                case LEFT_1: {
                    WritePosEx(2,1800,1000,100);
                    par.target_distance = 2370.0f;
                    pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                    par.degree_chassis = -625.0f;
                    if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                        par.degree_claw = -170.0f;
                        if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-230.0f){
                            WritePosEx(1,2300,1000,100);
                            osDelay(500);
                            Motor_State_Reset(&hDJI[0]);
                            Motor_State_Reset(&hDJI[2]);
                            stage_flag = 40;
                        }
                    }
                }break;
                case MIDDLE: {
                    WritePosEx(2,2048,1000,100);
                    par.target_distance = 2275.0f;
                    pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                    if(lidar.distance_aver>1800.0f){
                        par.degree_chassis = -541.0f;
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -190.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree>-230.0f){
                                WritePosEx(1,2300,1000,100);
                                osDelay(500);
                                Motor_State_Reset(&hDJI[0]);
                                Motor_State_Reset(&hDJI[2]);
                                stage_flag = 40;
                            }
                        }
                    }
                } break;
                case RIGHT_1: {
                    WritePosEx(2,2350,1000,100);
                    par.target_distance = 2365.0f;
                    if(lidar.distance_aver>1800.0f){
                        par.degree_chassis = -465.0f;
                        pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -170.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-200.0f){
                                WritePosEx(1,2100,1600,100);
                                osDelay(500);
                                Motor_State_Reset(&hDJI[0]);
                                Motor_State_Reset(&hDJI[2]);
                                stage_flag = 40;
                            }
                        }
                    }
                }break;
                case RIGHT_2: {
                    WritePosEx(2,2048,1000,100);
                    par.target_distance = 2655.0f;
                    if(lidar.distance_aver>700.0f){
                        pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                        par.degree_chassis = -329.0f;
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -170.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-200.0f){
                                WritePosEx(1,2100,1600,100);
                                osDelay(500);
                                Motor_State_Reset(&hDJI[0]);
                                Motor_State_Reset(&hDJI[2]);
                                stage_flag = 40;
                            }
                        }
                    }
                }break;
            }
        }
        if(stage_flag == 31){ //中间豆子放在最左边箱子后的特殊情况
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -450.0f){
                WritePosEx(1,2600,1000,100);
                WritePosEx(2,1725,1000,100);
                par.target_distance = 174.0f;
                if(lidar.distance_aver<2000.0f){
                    pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                    par.degree_chassis = -100.0f;
                    if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)){
                        stage_flag = 50;
                    }
                }
            }
        }
        if(stage_flag == 40){ //抓取左边豆子 //JIAOZHUN
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -400.0f){
                WritePosEx(1,2600,1000,100);
                WritePosEx(2,1725,1000,100);
                par.degree_chassis = -100.0f;
                pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                par.target_distance = 174.0f;
                if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)){
                    stage_flag = 50;
                }
            }
        }
        if(stage_flag == 50){//抓取
            par.degree_claw = -100.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-125.0f){
                WritePosEx(1,1900,1000,100);
                osDelay(1000);
                stage_flag = 60;
            }
        }
        if(stage_flag == 60){
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -500.0f){
                par.degree_chassis = -630.0f;
                pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                if(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<10.0f){
                    stage_flag = 70;
                    Motor_State_Reset(&hDJI[0]);
                    Motor_State_Reset(&hDJI[2]);
                }
            }
        }
        if(stage_flag == 70){
            switch(bean[1].target_position){
                case LEFT_2: {
                    WritePosEx(2,2048,1000,100);
                    par.target_distance = 2655.0f;
                    pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                    par.degree_chassis = -755.0f;
                    if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                        par.degree_claw = -170.0f;
                        if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-230.0f){
                            WritePosEx(1,2300,1000,100);
                            stage_flag = 80;//有点极限，退后一点再转向
                        }
                        }
                }break;
                case LEFT_1: {
                    WritePosEx(2,1800,1000,100);
                    par.target_distance = 2370.0f;
                    pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                    par.degree_chassis = -625.0f;
                    if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                        par.degree_claw = -170.0f;
                        if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-230.0f){
                            WritePosEx(1,2300,1000,100);
                            osDelay(500);
                            Motor_State_Reset(&hDJI[0]);
                            Motor_State_Reset(&hDJI[2]);
                            stage_flag = 80;
                        }
                    }
                }break;
                case MIDDLE: {
                    WritePosEx(2,2048,1000,100);
                    par.target_distance = 2275.0f;
                    pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                    if(lidar.distance_aver>1800.0f){
                        par.degree_chassis = -541.0f;
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -190.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree>-230.0f){
                                WritePosEx(1,2300,1000,100);
                                osDelay(500);
                                Motor_State_Reset(&hDJI[0]);
                                Motor_State_Reset(&hDJI[2]);
                                stage_flag = 80;
                            }
                        }
                    }
                } break;
                case RIGHT_1: {
                    WritePosEx(2,2350,1000,100);
                    par.target_distance = 2365.0f;
                    if(lidar.distance_aver>1800.0f){
                        par.degree_chassis = -465.0f;
                        pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -170.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-200.0f){
                                WritePosEx(1,2300,1000,100);
                                osDelay(500);
                                Motor_State_Reset(&hDJI[0]);
                                Motor_State_Reset(&hDJI[2]);
                                stage_flag = 80;
                            }
                        }

                    } 
                }break;
                case RIGHT_2: {
                    WritePosEx(2,2048,1000,100);
                    par.target_distance = 2655.0f;
                    if(lidar.distance_aver>1800.0f){
                        pid_reset(&hDJI[2], 10.0f, 0.0f, 0.0f);
                        par.degree_chassis = -329.0f;
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -170.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-200.0f){
                                        WritePosEx(1,2300,1000,100);
                                        osDelay(500);
                                        Motor_State_Reset(&hDJI[0]);
                                        Motor_State_Reset(&hDJI[2]);
                                        stage_flag = 71;  //有点极限，退后一点再转向
                            }
                        }
                    }
                }break;
            }
        }
        if(stage_flag == 71){ //左边豆子放最右边箱子的特殊情况
            par.degree_claw = -600.0f;
            WritePosEx(1,1850,1000,100);
            WritePosEx(2,1500,1000,100);
            if(hDJI[3].AxisData.AxisAngle_inDegree < -300.0f){
                par.target_distance = 189.0f;
                if(lidar.distance_aver<2000.0f){
                    pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                    par.degree_chassis = -983.0f;
                    if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)){
                        stage_flag = 80;
                    }
                }
            }
        }
        if(stage_flag == 80){ //抓取右边豆子
            par.degree_claw = -600.0f;
            WritePosEx(1,2800,1000,100);
            WritePosEx(2,2450,1000,100);
            if(hDJI[3].AxisData.AxisAngle_inDegree < -400.0f){
                par.degree_chassis = -983.0f;
                pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                par.target_distance = 189.0f;
                if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)){
                    stage_flag = 90;
                }
            }
        }
        if(stage_flag == 90){//向下抓取
            par.degree_claw = -150.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-190.0f){
                WritePosEx(1,1900,1000,100);
                osDelay(800);
                stage_flag = 100;
            }
        }
        if(stage_flag == 100){//抬升并旋转
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -400.0f){
                par.degree_chassis = -1250.0f;
                par.target_distance = 1600.0f;
                pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                if((hDJI[2].AxisData.AxisAngle_inDegree<-1200.0f)&&(lidar.distance_aver>1000.0f)){
                    stage_flag = 101;
                    Motor_State_Reset(&hDJI[2]);
                }
            }
        }
        if(stage_flag == 101){
            par.target_distance = 1600.0f;
            par.degree_chassis = -800.0f;
            if((lidar.distance_aver>700.0f)&&(hDJI[2].AxisData.AxisAngle_inDegree<-850.0f)){
                Motor_State_Reset(&hDJI[2]);
                Motor_State_Reset(&hDJI[0]);
                stage_flag = 110;
            }
        }
        if(stage_flag == 110){
            switch(bean[0].target_position){
                case LEFT_2: {
                    WritePosEx(2,2048,1000,100);
                    par.target_distance = 2655.0f;
                    pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                    par.degree_chassis = -755.0f;
                    if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                        par.degree_claw = -170.0f;
                        if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-230.0f){
                            WritePosEx(1,2300,1000,100);
                            while(1){};
                        }
                        }
                }break;
                case LEFT_1: {
                    WritePosEx(2,1800,1000,100);
                    par.target_distance = 2370.0f;
                    if(lidar.distance_aver>700.0f){
                        par.degree_chassis = -625.0f;
                        pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -170.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-200.0f){
                                WritePosEx(1,2300,1000,100);
                                while(1){};
                            }
                        }
                    }
                }break;
                case MIDDLE: {    
                    WritePosEx(2,2048,1000,100);
                    par.target_distance = 2275.0f;
                    pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                    if(lidar.distance_aver>700.0f){
                        par.degree_chassis = -541.0f;
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -190.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree>-230.0f){
                                WritePosEx(1,2300,1000,100);
                                osDelay(500);
                                Motor_State_Reset(&hDJI[0]);
                                Motor_State_Reset(&hDJI[2]);
                                while(1){};
                            }
                        }
                    }
                } break;
                case RIGHT_1: {
                    WritePosEx(2,2350,1000,100);
                    par.target_distance = 2365.0f;
                    if(lidar.distance_aver>1800.0f){
                        par.degree_chassis = -465.0f;
                        pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -170.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-200.0f){
                                WritePosEx(1,2100,1600,100);
                                osDelay(500);
                                Motor_State_Reset(&hDJI[0]);
                                Motor_State_Reset(&hDJI[2]);
                                while(1){};
                            }
                        }

                    } 
                }break;
                case RIGHT_2: {
                    WritePosEx(2,2048,1000,100);
                    par.target_distance = 2655.0f;
                    if(lidar.distance_aver>1500.0f){
                        pid_reset(&hDJI[2], 5.0f, 0.0f, 0.0f);
                        par.degree_chassis = -329.0f;
                        if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<1.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                            par.degree_claw = -170.0f;
                            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-200.0f){
                                WritePosEx(1,2100,1600,100);
                                osDelay(500);
                                Motor_State_Reset(&hDJI[0]);
                                Motor_State_Reset(&hDJI[2]);
                                while(1){};
                            }
                        }
                    }
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