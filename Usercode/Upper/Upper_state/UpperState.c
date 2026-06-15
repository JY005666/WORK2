#include "UpperState.h"
#include"stdio.h"
#include"stdlib.h"

#define CLAW_UP 1
#define CLAW_DOWN 2
#define CLAW_OPEN 106
#define CLAW_HALF_OPEN 70
#define CLAW_CLOSE 0


paramater par;

uint16_t stage_flag = 0;

//箱子和豆子（视觉识别）
Bean bean[3];
Box box[5]; 

//豆子位置和箱子位置对应的角度
Angle bean_left;
Angle bean_right;
Angle bean_middle;
Angle box_left_2;
Angle box_left_1;
Angle box_middle_0;
Angle box_right_1;
Angle box_right_2;

void Angle_Init(void){

    bean_left.distance = 174.0f;
    bean_left.chassis = -100.0f;
    bean_left.claw_angle = 120;

    bean_right.distance = 189.0f;
    bean_right.chassis = -983.0f;
    bean_right.claw_angle = 50;

    bean_middle.distance = 590.0f;
    bean_middle.chassis = -7.0f;
    bean_middle.claw_angle = 85;

    box_left_2.distance = 2743.0f;
    box_left_2.chassis = -775.0f;
    box_left_2.claw_angle = 70;

    box_left_1.distance = 2380.0f;
    box_left_1.chassis = -625.0f;
    box_left_1.claw_angle = 110;

    box_middle_0.distance = 2285.0f;
    box_middle_0.chassis = -541.0f;
    box_middle_0.claw_angle = 85;

    box_right_1.distance = 2375.0f;
    box_right_1.chassis = -465.0f;
    box_right_1.claw_angle = 55;

    box_right_2.distance = 2755.0f;
    box_right_2.chassis = -297.0f;
    box_right_2.claw_angle = 100;
}

void Bean_Init(void){
    bean[0].position = RIGHT;
    bean[1].position = LEFT;
    bean[2].position = MIDDLE; 

    bean[0].target_position = LEFT_1;
    bean[1].target_position = RIGHT_1;
    bean[2].target_position = RIGHT_2;
} 
void Bean_Target_Set(void){
    for(int i=0;i<3;i++){
        switch(g_pos[2-i]){
            case 1: bean[i].target_position = LEFT_2; break;
            case 2: bean[i].target_position = LEFT_1; break;
            case 3: bean[i].target_position = MIDDLE_0; break;
            case 4: bean[i].target_position = RIGHT_1; break;
            case 5: bean[i].target_position = RIGHT_2; break;
        }
    }
}
void init_paramater(paramater *par){
    par->target_distance = lidar.distance_aver;
    par->degree_chassis = 0.0;
    par->degree_claw = 0.0;
}

/* 提取出的箱子放置 switch，三段重复代码合并 */
static void Bean_Place_Switch(Bean *b, uint16_t next_normal, uint16_t next_special)
{
    switch(b->target_position){
        case LEFT_2: {
            Claw_degree_set(box_left_2.claw_angle,CLAW_UP);
            par.target_distance = box_left_2.distance;
            par.degree_chassis = box_left_2.chassis;
            if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<8.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                par.degree_claw = -170.0f;
                if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-230.0f){
                    Claw_degree_set(CLAW_HALF_OPEN,CLAW_DOWN);
                    osDelay(500);
                    Motor_State_Reset(&hDJI[0]);
                    Motor_State_Reset(&hDJI[2]);
                    if(b->position == MIDDLE ){
                        stage_flag = next_special;
                    }
                    else stage_flag = next_normal;

                }
            }
        }break;
        case LEFT_1: {
            Claw_degree_set(box_left_1.claw_angle,CLAW_UP);
            par.target_distance = box_left_1.distance;
            par.degree_chassis = box_left_1.chassis;
            if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<8.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                par.degree_claw = -170.0f;
                if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-230.0f){
                    Claw_degree_set(CLAW_HALF_OPEN,CLAW_DOWN);
                    osDelay(500);
                    Motor_State_Reset(&hDJI[0]);
                    Motor_State_Reset(&hDJI[2]);
                    stage_flag = next_normal;
                }
            }
        }break;
        case MIDDLE_0: {
            Claw_degree_set(box_middle_0.claw_angle,CLAW_UP);
            par.target_distance = box_middle_0.distance;
            if(lidar.distance_aver>1800.0f){
                par.degree_chassis = box_middle_0.chassis;
                if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<8.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                    par.degree_claw = -190.0f;
                    if(hDJI[3].AxisData.AxisAngle_inDegree>-230.0f){
                        Claw_degree_set(CLAW_HALF_OPEN,CLAW_DOWN);
                        osDelay(500);
                        Motor_State_Reset(&hDJI[0]);
                        Motor_State_Reset(&hDJI[2]);
                        stage_flag = next_normal;
                    }
                }
            }
        } break;
        case RIGHT_1: {
            Claw_degree_set(box_right_1.claw_angle,CLAW_UP);
            par.target_distance = box_right_1.distance;
            if(lidar.distance_aver>1800.0f){
                par.degree_chassis = box_right_1.chassis;
                if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<8.0f)&&(abs(lidar.distance_aver-par.target_distance)<5.0f)){
                    par.degree_claw = -170.0f;
                    if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-200.0f){
                        Claw_degree_set(CLAW_HALF_OPEN,CLAW_DOWN);
                        osDelay(500);
                        Motor_State_Reset(&hDJI[0]);
                        Motor_State_Reset(&hDJI[2]);
                        stage_flag = next_normal;
                    }
                }
            }
        }break;
        case RIGHT_2: {
            Claw_degree_set(box_right_2.claw_angle,CLAW_UP);
            if(lidar.distance_aver<2000.0f){
                par.target_distance = 2155.0f;
            }
            if(lidar.distance_aver>2000.0f){
                par.degree_chassis = box_right_2.chassis;
                if((abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<20.0f)){
                    par.target_distance = box_right_2.distance;
                    if(abs(lidar.distance_aver-par.target_distance)<5.0f){
                        par.degree_claw = -190.0f;
                        if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-230.0f){
                            Claw_degree_set(CLAW_HALF_OPEN,CLAW_DOWN);
                            osDelay(500);
                            Motor_State_Reset(&hDJI[0]);
                            Motor_State_Reset(&hDJI[2]);
                            if(b->position == LEFT){
                                stage_flag = next_special;
                            } 
                            else stage_flag = next_normal;
                        }
                    }
                }
            }
        }break;
    }
}

void Upper_State_Task(void *arg){
    for(;;){
        if(stage_flag == 0){//到达中间豆子抓取位置，张开爪子
            if(lidar.distance_aver>700.0){
                pid_reset(&hDJI[3], 5.0f, 0.0f, 0.0f);
                par.degree_claw = -600.0f;
                par.degree_chassis = 60.0f;
                par.target_distance = bean_middle.distance;
            }
            if(lidar.distance_aver < 700.0){
                Claw_degree_set(CLAW_OPEN,CLAW_DOWN);
                Claw_degree_set(bean_middle.claw_angle,CLAW_UP);
                par.degree_chassis = bean_middle.chassis;
                if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)){
                    stage_flag = 10;
                }
            }
        }
        if(stage_flag == 10){//向下抓取
            osDelay(300);
            par.degree_claw = -250.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree>-400.0f){
                Claw_degree_set(CLAW_CLOSE,CLAW_DOWN);
                osDelay(800);
                Motor_State_Reset(&hDJI[2]);
                Motor_State_Reset(&hDJI[0]);
                stage_flag = 20;
            }
        }
                if(stage_flag == 20){//抬升并旋转
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -580.0f){
                // Motor_State_Reset(&hDJI[2]);
                par.degree_chassis = -630.0f;
                if(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<30.0f){
                    stage_flag = 30;
                    Motor_State_Reset(&hDJI[0]);
                    // Motor_State_Reset(&hDJI[2]);
                }
            }
        }
                if(stage_flag == 30){
            Bean_Place_Switch(&bean[2], 40, 31);
        }
        if(stage_flag == 31){ //中间豆子放在最左边箱子后的特殊情况
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -450.0f){
                Claw_degree_set(CLAW_OPEN,CLAW_DOWN);
                Claw_degree_set(bean_left.claw_angle,CLAW_UP);
                par.target_distance = bean_left.distance;
                if(lidar.distance_aver<2000.0f){
                    par.degree_chassis = bean_left.chassis;
                    if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)){
                        stage_flag = 50;
                    }
                }
            }
        }
        if(stage_flag == 40){ //抓取左边豆子 
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -400.0f){
                Claw_degree_set(CLAW_OPEN,CLAW_DOWN);
                Claw_degree_set(bean_left.claw_angle,CLAW_UP);
                par.degree_chassis = bean_left.chassis;
                par.target_distance = bean_left.distance;
                if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)){
                    stage_flag = 50;
                }
            }
        }
        if(stage_flag == 50){//向下抓取
            par.degree_claw = -50.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-90.0f){
                Claw_degree_set(CLAW_CLOSE,CLAW_DOWN);
                osDelay(800);
                stage_flag = 60;
            }
        }
        if(stage_flag == 60){
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -550.0f){
                par.degree_chassis = -630.0f;
                if(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<30.0f){
                    stage_flag = 70;
                    Motor_State_Reset(&hDJI[0]);
                    Motor_State_Reset(&hDJI[2]);
                }
            }
        }
                if(stage_flag == 70){
            Bean_Place_Switch(&bean[1], 80, 71);
        }
        if(stage_flag == 71){ //左边豆子放最右边箱子的特殊情况
            par.degree_claw = -600.0f;
            Claw_degree_set(CLAW_OPEN,CLAW_DOWN);
            Claw_degree_set(bean_right.claw_angle,CLAW_UP);
            if(hDJI[3].AxisData.AxisAngle_inDegree < -300.0f){
                par.target_distance = bean_right.distance;
                if(lidar.distance_aver<2000.0f){
                    par.degree_chassis = bean_right.chassis;
                    if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)){
                        stage_flag = 80;
                    }
                }
            }
        }
        if(stage_flag == 80){ //抓取右边豆子
            par.degree_claw = -600.0f;
            Claw_degree_set(CLAW_OPEN,CLAW_DOWN);
            Claw_degree_set(bean_right.claw_angle,CLAW_UP);
            if(hDJI[3].AxisData.AxisAngle_inDegree < -580.0f){
                par.degree_chassis = bean_right.chassis;
                par.target_distance = bean_right.distance;
                if((abs(lidar.distance_aver-par.target_distance)<5.0f)&&(abs(hDJI[2].AxisData.AxisAngle_inDegree-par.degree_chassis)<2.0f)){
                    stage_flag = 90;
                }
            }
        }
        if(stage_flag == 90){//向下抓取
            par.degree_claw = -100.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree-par.degree_claw>-120.0f){
                Claw_degree_set(CLAW_CLOSE,CLAW_DOWN);
                osDelay(800);
                stage_flag = 100;
            }
        }
        if(stage_flag == 100){//抬升并旋转
            par.degree_claw = -600.0f;
            if(hDJI[3].AxisData.AxisAngle_inDegree < -450.0f){
                par.degree_chassis = -1250.0f;
                par.target_distance = 1600.0f;
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
            Bean_Place_Switch(&bean[0], 800, 710);
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