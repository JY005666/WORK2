#include "UpperState.h"
#include "StateHelpers/UpperStateHelpers.h"

#include "math.h"
#include "stdio.h"
#include "stdlib.h"

paramater par;

uint16_t stage_flag = 0;

// 箱子和豆子（视觉识别）
Bean bean[3];
Box box[5];

// 豆子位置和箱子位置对应的角度
Angle bean_left;
Angle bean_right;
Angle bean_middle;
Angle box_left_2;
Angle box_left_1;
Angle box_middle_0;
Angle box_right_1;
Angle box_right_2;
float box_middle_0_chassis_cw;
float box_middle_0_chassis_ccw;
BeanPosition g_second_bean_position = MIDDLE;
BeanPosition g_third_bean_position = MIDDLE;


static void HandleStage0(void);
static void HandleStage10(void);
static void HandleBeanDelivery(BeanPosition bean_position, uint16_t next_stage);



void Angle_Init(void);

void Bean_Init(void);

void Bean_Target_Set(void);

void init_paramater(paramater *par);

void Upper_State_Task(void *arg)
{
    for (;;) {
        switch (stage_flag) {
            case 0:
                HandleStage0(); //到达第一个抓取位置
                break;
            case 10:
                HandleStage10(); //抓取第一个豆子
                break;
            case 20:  //上升爪子到打不到箱子
                par.degree_claw = MIDDLE_BEAN_DELIVERY_LIFT_TARGET_DEG;
                if (hDJI[3].AxisData.AxisAngle_inDegree < MIDDLE_BEAN_DELIVERY_LIFT_READY_DEG) {
                    ResetStage30PlacementState();
                    stage_flag = 30;
                }; 
                break;
            case 30:
                HandleStage30_FirstBeanPlacement(); //放置第一个豆子
                break;
            case 31:
                HandleStage31_SecondBeanPickup(); //判断第二个豆子是哪个,并到达抓取位置
                break;
            case 40:
                if (IsDistanceAndChassisReady(10.0f, 3.0f)) { //判断是否到达第二个豆子抓取位置
                    YawServo_ForceLockCurrent(&hDJI[2]);
                    osDelay(20);
                    stage_flag = 50;
                }
                break;
            case 50: //下降夹爪并夹取
                if (g_second_bean_position == LEFT) {
                    par.degree_claw = -90.0f;
                    if (hDJI[3].AxisData.AxisAngle_inDegree > -100.0f) {
                        CloseClawAndAdvance(60);
                    }; 
                }
                if (g_second_bean_position == RIGHT) {
                    par.degree_claw = -180.0f;
                    if (hDJI[3].AxisData.AxisAngle_inDegree  > -190.0f) {
                        CloseClawAndAdvance(60);
                    }
                }
                break;
            case 60: //将爪子上升到安全高度，避免送箱过程中碰到障碍物
                ResetDistanceAndChassisMotors();
                if (g_second_bean_position == LEFT) {
                    par.degree_claw = LEFT_BEAN_DELIVERY_LIFT_TARGET_DEG;
                    if (hDJI[3].AxisData.AxisAngle_inDegree < LEFT_BEAN_DELIVERY_LIFT_READY_DEG) {
                        stage_flag = 61;
                    }; 
                }
                if (g_second_bean_position == RIGHT) {
                    par.degree_claw = RIGHT_BEAN_DELIVERY_LIFT_TARGET_DEG;
                    if (hDJI[3].AxisData.AxisAngle_inDegree < RIGHT_BEAN_DELIVERY_LIFT_READY_DEG) {
                        stage_flag = 61;
                    }
                }
                break;
            case 61: //避障与设计送箱目标值
                HandleBeanDelivery(g_second_bean_position, 70);
                break;
            case 70: //放下第二个豆子
                par.degree_claw = DELIVERY_RELEASE_LIFT_TARGET_DEG;
                if (hDJI[3].AxisData.AxisAngle_inDegree  > DELIVERY_RELEASE_LIFT_READY_DEG) {
                    // YawServo_ForceLockCurrent(&hDJI[2]);
                    osDelay(20);
                    Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                    osDelay(800);
                    ResetDistanceAndChassisMotors();
                    stage_flag = 900;
                }
                break;
            case 900: //到达第三个豆子抓取位置
                par.degree_claw = -600.0f;
                Claw_degree_set(CLAW_OPEN, CLAW_DOWN);
                if(g_third_bean_position == RIGHT){
                    par.degree_chassis = bean_right.chassis;
                    par.target_distance = bean_right.distance;
                    Claw_degree_set(bean_right.claw_angle, CLAW_UP);
                }
                else if(g_third_bean_position == LEFT){
                    par.degree_chassis = bean_left.chassis;
                    par.target_distance = bean_left.distance;
                    Claw_degree_set(bean_left.claw_angle, CLAW_UP);
                }
                if (IsDistanceAndChassisReady(10.0f, 3.0f)) { //判断是否到达第三个豆子抓取位置
                    YawServo_ForceLockCurrent(&hDJI[2]);
                    osDelay(20);
                    stage_flag = 910;
                }
                break;
            case 910: // 下降夹爪并夹取第三个豆子
                if (g_third_bean_position == LEFT) {
                    par.degree_claw = -90.0f;
                    if (hDJI[3].AxisData.AxisAngle_inDegree > -100.0f) {
                        CloseClawAndAdvance(911);
                    }; 
                }
                if (g_third_bean_position == RIGHT) {
                    par.degree_claw = -180.0f;
                    if (hDJI[3].AxisData.AxisAngle_inDegree  > -190.0f) {
                        CloseClawAndAdvance(911);
                    }
                }
                break;
            case 911: //将爪子上升到安全高度，避免送箱过程中碰到障碍物
                ResetDistanceAndChassisMotors();
                if (g_third_bean_position == LEFT) {
                    par.degree_claw = LEFT_BEAN_DELIVERY_LIFT_TARGET_DEG;
                    if (hDJI[3].AxisData.AxisAngle_inDegree < LEFT_BEAN_DELIVERY_LIFT_READY_DEG) {
                        stage_flag = 920;
                    }; 
                }
                if (g_third_bean_position == RIGHT) {
                    par.degree_claw = RIGHT_BEAN_DELIVERY_LIFT_TARGET_DEG;
                    if (hDJI[3].AxisData.AxisAngle_inDegree < RIGHT_BEAN_DELIVERY_LIFT_READY_DEG) {
                        stage_flag = 920;
                    }
                }
                break;
            case 920:
                HandleBeanDelivery(g_third_bean_position, 930);
                break;
            case 930:
                par.degree_claw = DELIVERY_RELEASE_LIFT_TARGET_DEG;
                if (hDJI[3].AxisData.AxisAngle_inDegree  > DELIVERY_RELEASE_LIFT_READY_DEG) {
                    Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                    osDelay(800);
                    ResetDistanceAndChassisMotors();
                    stage_flag = 1000;
                }
                break;
            default:
                break;
        }

        osDelay(2);
    }
}

void DebugPrint(void)
{
    static uint32_t s_last_print_tick = 0U;
    static uint16_t s_last_stage = 0xFFFFU;
    static float s_last_target_distance = 0.0f;
    static float s_last_target_chassis = 0.0f;
    static float s_last_control_distance = 0.0f;
    uint32_t now_tick = HAL_GetTick();
    uint8_t should_print = 0U;

    if (stage_flag != s_last_stage) {
        should_print = 1U;
    }

    if (fabsf(par.target_distance - s_last_target_distance) > 2.0f ||
        fabsf(par.degree_chassis - s_last_target_chassis) > 1.0f) {
        should_print = 1U;
    }

    if (fabsf(g_distance_servo_debug.control_distance - s_last_control_distance) > 5.0f) {
        should_print = 1U;
    }

    if ((now_tick - s_last_print_tick) >= 100U) {
        should_print = 1U;
    }

    if (!should_print) {
        return;
    }

    printf("stage:%u,target_distance:%.1f,lidar_live:%.1f,raw:%.1f,reliable:%.1f,filtered:%.1f,control:%.1f,error:%.1f,stop_error:%.1f,speed_ref:%.1f,motor_rpm:%.1f,near:%u,arrived:%u,arrived_confirmed:%u,target_chassis:%.1f,chassis:%.1f,claw:%.1f\r\n",
           stage_flag,
           g_distance_servo_debug.target_distance,
           lidar.distance_aver,
           g_distance_servo_debug.raw_distance,
           g_distance_servo_debug.reliable_distance,
           g_distance_servo_debug.filtered_distance,
           g_distance_servo_debug.control_distance,
           g_distance_servo_debug.error_distance,
           g_distance_servo_debug.stop_error_distance,
           g_distance_servo_debug.desired_speed_ref,
           g_distance_servo_debug.motor_rpm,
           g_distance_servo_debug.use_reliable_near_target,
           g_distance_servo_debug.arrived,
           g_distance_servo_debug.arrived_confirmed,
           par.degree_chassis,
           hDJI[2].AxisData.AxisAngle_inDegree,
           hDJI[3].AxisData.AxisAngle_inDegree);

    s_last_print_tick = now_tick;
    s_last_stage = stage_flag;
    s_last_target_distance = par.target_distance;
    s_last_target_chassis = par.degree_chassis;
    s_last_control_distance = g_distance_servo_debug.control_distance;
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
static void HandleStage0(void)
{
    par.degree_claw = -650.0f;
    if(hDJI[3].AxisData.AxisAngle_inDegree<-50.0f){
        if (lidar.distance_aver > 1100.0f) {
            par.degree_chassis = 50.0f;
            par.target_distance = bean_middle.distance;
        }
        if (lidar.distance_aver < 1000.0f) {
            Claw_degree_set(CLAW_OPEN, CLAW_DOWN);
            Claw_degree_set(bean_middle.claw_angle, CLAW_UP);

            par.degree_chassis = bean_middle.chassis;
            if (IsDistanceAndChassisReady(10.0f, 3.0f)) {
                osDelay(300);
                stage_flag = 10;
            }
        }
    }

}
static void HandleStage10(void)
{
    par.degree_claw = -285.0f;
    if (hDJI[3].AxisData.AxisAngle_inDegree > -295.0f) {
        Claw_degree_set(CLAW_CLOSE, CLAW_DOWN);
        osDelay(800);
        ResetDistanceAndChassisMotors();
        stage_flag = 20;
    }
}

void Angle_Init(void)
{
    bean_left.distance = 250.0f;
    bean_left.chassis = -96.0f;
    bean_left.claw_angle = 123;

    bean_right.distance = 265.0f;
    bean_right.chassis = 88.0f;
    bean_right.claw_angle = 63;

    bean_middle.distance = 657.0f;
    bean_middle.chassis = -3.0f;
    bean_middle.claw_angle = 93;

    box_left_2.distance = 2470.0f;
    box_left_2.chassis = 345.0f;
    box_left_2.claw_angle = 66;

    box_left_1.distance = 2243.0f;
    box_left_1.chassis = 460.0f;
    box_left_1.claw_angle = 115;

    box_middle_0.distance = 2160.0f;
    box_middle_0_chassis_cw = 538.0f;
    box_middle_0_chassis_ccw = -548.0f;
    box_middle_0.claw_angle = 93;

    box_right_1.distance = 2235.0f;
    box_right_1.chassis = -473.0f;
    box_right_1.claw_angle = 65;

    box_right_2.distance = 2435.0f;
    box_right_2.chassis = -353.0f;
    box_right_2.claw_angle = 117;
}

void Bean_Init(void)
{
    bean[0].position = RIGHT;
    bean[1].position = LEFT;
    bean[2].position = MIDDLE;

    bean[0].target_position = LEFT_2;
    bean[1].target_position = RIGHT_1;
    bean[2].target_position = MIDDLE_0;
}

void Bean_Target_Set(void)
{
    for (int i = 0; i < 3; i++) {
        switch (g_pos[2 - i]) {
            case 1: bean[i].target_position = LEFT_2; break;
            case 2: bean[i].target_position = LEFT_1; break;
            case 3: bean[i].target_position = MIDDLE_0; break;
            case 4: bean[i].target_position = RIGHT_1; break;
            case 5: bean[i].target_position = RIGHT_2; break;
        }
    }
}

void init_paramater(paramater *par)
{
    par->target_distance = lidar.distance_aver;
    par->degree_chassis = 0.0f;
    par->degree_claw = 0.0f;
}
void HandleBeanDelivery(BeanPosition bean_position, uint16_t next_stage){
    if(bean_position == LEFT){ //左边豆子
        switch (bean[1].target_position) {
            case LEFT_2: par.target_distance = box_left_2.distance; Claw_degree_set(box_left_2.claw_angle, CLAW_UP); break;
            case LEFT_1: par.target_distance = box_left_1.distance; Claw_degree_set(box_left_1.claw_angle, CLAW_UP); break;
            case MIDDLE_0: par.target_distance = box_middle_0.distance; Claw_degree_set(box_middle_0.claw_angle, CLAW_UP); break;
            case RIGHT_1: par.target_distance = box_right_1.distance; Claw_degree_set(box_right_1.claw_angle, CLAW_UP); break;
            case RIGHT_2: par.target_distance = box_right_2.distance; Claw_degree_set(box_right_2.claw_angle, CLAW_UP); break;
        }
        if(bean[1].target_position == LEFT_2||bean[1].target_position == LEFT_1||bean[1].target_position == MIDDLE_0){
            if(lidar.distance_aver > SAFE_DIST_FOR_BOX_FINAL_TURN_MM-200){
                // if(bean[1].target_position != RIGHT_2&&bean[1].target_position != LEFT_2){par.degree_claw = DELIVERY_RELEASE_LIFT_TARGET_DEG;}
                //设置云台目标角度
                switch (bean[1].target_position) {
                    case LEFT_2: par.degree_chassis = box_left_2.chassis; break;
                    case LEFT_1: par.degree_chassis = box_left_1.chassis; break;
                    case MIDDLE_0: par.degree_chassis = box_middle_0_chassis_cw; break;
                    case RIGHT_1: par.degree_chassis = box_right_1.chassis; break;
                    case RIGHT_2: par.degree_chassis = box_right_2.chassis; break;
                }
            }
        }
        else{ 
            if(lidar.distance_aver < SAFE_DIST_FOR_BOX_FINAL_TURN_MM){
                par.degree_chassis = FIRST_BEAN_RIGHT_SAFE_CW_DEG ;
            } else
            if(lidar.distance_aver > SAFE_DIST_FOR_BOX_FINAL_TURN_MM){
                // if(bean[1].target_position != RIGHT_2&&bean[1].target_position != LEFT_2){par.degree_claw = DELIVERY_RELEASE_LIFT_TARGET_DEG;}
                switch (bean[1].target_position) {
                    case LEFT_2: par.degree_chassis = box_left_2.chassis; break;
                    case LEFT_1: par.degree_chassis = box_left_1.chassis; break;
                    case MIDDLE_0: par.degree_chassis = box_middle_0_chassis_cw; break;
                    case RIGHT_1: par.degree_chassis = box_right_1.chassis; break;
                    case RIGHT_2: par.degree_chassis = box_right_2.chassis; break;
                }
                }
            }
    }else //右边豆子
    if(bean_position == RIGHT){
        switch (bean[0].target_position) {
            case LEFT_2: par.target_distance = box_left_2.distance; Claw_degree_set(box_left_2.claw_angle, CLAW_UP); break;
            case LEFT_1: par.target_distance = box_left_1.distance; Claw_degree_set(box_left_1.claw_angle, CLAW_UP); break;
            case MIDDLE_0: par.target_distance = box_middle_0.distance; Claw_degree_set(box_middle_0.claw_angle, CLAW_UP); break;
            case RIGHT_1: par.target_distance = box_right_1.distance; Claw_degree_set(box_right_1.claw_angle, CLAW_UP); break;
            case RIGHT_2: par.target_distance = box_right_2.distance; Claw_degree_set(box_right_2.claw_angle, CLAW_UP); break;
        }
        if(bean[0].target_position == RIGHT_2||bean[0].target_position == RIGHT_1||bean[0].target_position == MIDDLE_0){
            if(lidar.distance_aver > SAFE_DIST_FOR_BOX_FINAL_TURN_MM-200){
                // if(bean[0].target_position != RIGHT_2&&bean[0].target_position != LEFT_2){par.degree_claw = DELIVERY_RELEASE_LIFT_TARGET_DEG;}
                switch (bean[0].target_position) {
                    case RIGHT_1: par.degree_chassis = box_right_1.chassis; break;
                    case RIGHT_2: par.degree_chassis = box_right_2.chassis; break;
                    case MIDDLE_0: par.degree_chassis = box_middle_0_chassis_ccw; break;
                    case LEFT_1: par.degree_chassis = box_left_1.chassis; break;
                    case LEFT_2: par.degree_chassis = box_left_2.chassis; break;
                }

            }
        }
        else{
            if(lidar.distance_aver < SAFE_DIST_FOR_BOX_FINAL_TURN_MM){
                par.degree_chassis = FIRST_BEAN_LEFT_SAFE_CCW_DEG ;
            } else
            if(lidar.distance_aver > SAFE_DIST_FOR_BOX_FINAL_TURN_MM){
                // if(bean[0].target_position != RIGHT_2&&bean[0].target_position != LEFT_2){par.degree_claw = DELIVERY_RELEASE_LIFT_TARGET_DEG;}
                switch (bean[0].target_position) {
                    case RIGHT_1: par.degree_chassis = box_right_1.chassis; break;
                    case RIGHT_2: par.degree_chassis = box_right_2.chassis; break;
                    case MIDDLE_0: par.degree_chassis = box_middle_0_chassis_ccw; break;
                    case LEFT_1: par.degree_chassis = box_left_1.chassis; break;
                    case LEFT_2: par.degree_chassis = box_left_2.chassis; break;
                }
            }
        }
    }
    if(IsDistanceAndChassisReady(15.0f, 8.0f)){
        // YawServo_ForceLockCurrent(&hDJI[2]);
        osDelay(20);
        stage_flag = next_stage;
    }
}
