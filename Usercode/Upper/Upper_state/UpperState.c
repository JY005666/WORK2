#include "UpperState.h"
#include "StateHelpers/UpperStateHelpers.h"

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
ActiveBeanState g_active_bean_state = ACTIVE_NONE;
BoxPosition g_last_placed_box_position = MIDDLE_0;
BeanPosition g_third_bean_position = MIDDLE;
uint8_t g_delivery_distance_enabled = 0U;
uint8_t g_delivery_final_turn_enabled = 0U;

static void HandleStage0(void);
static void HandleStage10(void);
static void HandleBeanGrabFinishAndAdvance(uint16_t next_stage);

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
                if (ApplyDeliveryLiftGate(MIDDLE)) {
                    stage_flag = 30;
                } 
                break;
            case 30:
                HandleStage30_FirstBeanPlacement(); //放置第一个豆子
                break;
            case 31:
                HandleStage31_SecondBeanPickup(); //到达第二个豆子抓取位置
                break;
            case 50:
                HandleBeanGrabFinishAndAdvance(60); //抓取第二个豆子
                break;
            case 60:
                HandleActiveBeanDelivery(70, 110); //第二个豆子的送箱与障碍规避
                break;
            case 70:
                Bean_Place_Switch(&bean[1], 900); //左豆子
                break;
            case 110:
                Bean_Place_Switch(&bean[0], 900); //右豆子
                break;
            case 900:
                HandleStage900_ThirdBeanPickup(); //到达第三个豆子抓取位置
                break;
            case 910:
                HandleBeanGrabFinishAndAdvance(920); //抓取第三个豆子
                break;
            case 920:
                HandleActiveBeanDelivery(930, 940); //第三个豆子的送箱与障碍规避
                break;
            case 930:
                Bean_Place_Switch(&bean[1], 1000);
                break;
            case 940:
                Bean_Place_Switch(&bean[0], 1000);
                break;
            default:
                break;
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
static void HandleStage0(void)
{
    par.degree_claw = -600.0f;
    if(hDJI[3].AxisData.AxisAngle_inDegree<-5.0f){
        if (lidar.distance_aver > 700.0f) {
            par.degree_chassis = 30.0f;
            par.target_distance = bean_middle.distance;
        }
        if (lidar.distance_aver < 700.0f) {
            Claw_degree_set(CLAW_OPEN, CLAW_DOWN);
            Claw_degree_set(bean_middle.claw_angle, CLAW_UP);

            par.degree_chassis = bean_middle.chassis;
            if (IsDistanceAndChassisReady(5.0f, 0.5f)) {
                osDelay(300);
                stage_flag = 10;
            }
        }
    }

}
static void HandleStage10(void)
{

    par.degree_claw = -250.0f;
    if (hDJI[3].AxisData.AxisAngle_inDegree > -375.0f) {
        Claw_degree_set(CLAW_CLOSE, CLAW_DOWN);
        osDelay(800);
        ResetDistanceAndChassisMotors();
        stage_flag = 20;
    }
}
static void HandleBeanGrabFinishAndAdvance(uint16_t next_stage)
{
    if (g_active_bean_state == ACTIVE_BEAN_LEFT) {
        par.degree_claw = -100.0f;
        if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -150.0f) {
            g_delivery_distance_enabled = 0U;
            g_delivery_final_turn_enabled = 0U;
            CloseClawAndAdvance(next_stage);
        }
        return;
    }

    if (g_active_bean_state == ACTIVE_BEAN_RIGHT) {
        par.degree_claw = -100.0f;
        if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -120.0f) {
            g_delivery_distance_enabled = 0U;
            g_delivery_final_turn_enabled = 0U;
            CloseClawAndAdvance(next_stage);
        }
    }
}
void Angle_Init(void)
{
    bean_left.distance = 310.0f;
    bean_left.chassis = -95.0f;
    bean_left.claw_angle = 115;

    bean_right.distance = 300.0f;
    bean_right.chassis = 85.0f;
    bean_right.claw_angle = 53;

    bean_middle.distance = 687.0f;
    bean_middle.chassis = -4.0f;
    bean_middle.claw_angle = 85;

    box_left_2.distance = 2500.0f;
    box_left_2.chassis = 345.0f;
    box_left_2.claw_angle = 58;

    box_left_1.distance = 2273.0f;
    box_left_1.chassis = 461.0f;
    box_left_1.claw_angle = 105;

    box_middle_0.distance = 2180.0f;
    box_middle_0_chassis_cw = 537.0f;
    box_middle_0_chassis_ccw = -548.0f;
    box_middle_0.claw_angle = 85;

    box_right_1.distance = 2275.0f;
    box_right_1.chassis = -470.0f;
    box_right_1.claw_angle = 60;

    box_right_2.distance = 2495.0f;
    box_right_2.chassis = -354.0f;
    box_right_2.claw_angle = 112;
}

void Bean_Init(void)
{
    bean[0].position = RIGHT;
    bean[1].position = LEFT;
    bean[2].position = MIDDLE;

    bean[0].target_position = RIGHT_1;
    bean[1].target_position = LEFT_1;
    bean[2].target_position = RIGHT_1;
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
