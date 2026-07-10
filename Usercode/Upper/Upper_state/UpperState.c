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

static void HandleStage0(void)
{
    if (lidar.distance_aver > 700.0f) {
        pid_reset(&hDJI[3], 5.0f, 0.0f, 0.0f);
        par.degree_claw = -600.0f;
        par.degree_chassis = 60.0f;
        par.target_distance = bean_middle.distance;
    }
    if (lidar.distance_aver < 700.0f) {
        Claw_degree_set(CLAW_OPEN, CLAW_DOWN);
        Claw_degree_set(bean_middle.claw_angle, CLAW_UP);
        par.degree_chassis = bean_middle.chassis;
        if (IsDistanceAndChassisReady(5.0f, 0.5f)) {
            stage_flag = 10;
        }
    }
}

static void HandleStage10(void)
{
    osDelay(300);
    par.degree_claw = -250.0f;
    if (hDJI[3].AxisData.AxisAngle_inDegree > -400.0f) {
        Claw_degree_set(CLAW_CLOSE, CLAW_DOWN);
        osDelay(500);
        ResetDistanceAndChassisMotors();
        stage_flag = 20;
    }
}

static void HandleStage20(void)
{
    par.degree_claw = -650.0f;
    if (hDJI[3].AxisData.AxisAngle_inDegree < -580.0f) {
        stage_flag = 30;
    }
}

static void HandleStage50(void)
{
    par.degree_claw = -50.0f;
    if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -90.0f) {
        CloseClawAndAdvance(60);
    }
}

static void HandleStage60(void)
{
    LiftAndRotateToPlacement(-650.0f, -550.0f, -630.0f, 30.0f, 70);
}

static void HandleStage90(void)
{
    par.degree_claw = -100.0f;
    if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -120.0f) {
        CloseClawAndAdvance(100);
    }
}

static void HandleStage100(void)
{
    par.degree_claw = -650.0f;
    if (hDJI[3].AxisData.AxisAngle_inDegree < -500.0f) {
        par.degree_chassis = -1250.0f;
        par.target_distance = 1600.0f;
        if ((hDJI[2].AxisData.AxisAngle_inDegree < -1200.0f) && (lidar.distance_aver > 1000.0f)) {
            stage_flag = 101;
            Motor_State_Reset(&hDJI[2]);
        }
    }
}

static void HandleStage101(void)
{
    par.target_distance = 1600.0f;
    par.degree_chassis = -800.0f;
    if ((lidar.distance_aver > 700.0f) && (hDJI[2].AxisData.AxisAngle_inDegree < -850.0f)) {
        ResetDistanceAndChassisMotors();
        stage_flag = 110;
    }
}

void Angle_Init(void)
{
    bean_left.distance = 165.0f;
    bean_left.chassis = -105.0f;
    bean_left.claw_angle = 120;

    bean_right.distance = 189.0f;
    bean_right.chassis = -983.0f;
    bean_right.claw_angle = 50;

    bean_middle.distance = 656.0f;
    bean_middle.chassis = -6.0f;
    bean_middle.claw_angle = 85;

    box_left_2.distance = 2400.0f;
    box_left_2.chassis = 350.0f;
    box_left_2.claw_angle = 70;

    box_left_1.distance = 2200.0f;
    box_left_1.chassis = 475.0f;
    box_left_1.claw_angle = 110;

    box_middle_0.distance = 2295.0f;
    box_middle_0.chassis = -541.0f;
    box_middle_0_chassis_cw = 525.0f;
    box_middle_0_chassis_ccw = -545.0f;
    box_middle_0.claw_angle = 85;

    box_right_1.distance = 2200.0f;
    box_right_1.chassis = -460.0f;
    box_right_1.claw_angle = 55;

    box_right_2.distance = 2400.0f;
    box_right_2.chassis = -360.0f;
    box_right_2.claw_angle = 100;
}

void Bean_Init(void)
{
    bean[0].position = RIGHT;
    bean[1].position = LEFT;
    bean[2].position = MIDDLE;

    bean[0].target_position = LEFT_1;
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

void Upper_State_Task(void *arg)
{
    for (;;) {
        switch (stage_flag) {
            case 0:
                HandleStage0();
                break;
            case 10:
                HandleStage10();
                break;
            case 20:
                HandleStage20();
                break;
            case 30:
                HandleStage30_FirstBeanPlacement();
                break;
            case 31:
                HandleStage31_SecondBeanPickup();
                break;
            case 40:
                HandleStage31Or40(50, -400.0f, 0U);
                break;
            case 50:
                HandleStage50();
                break;
            case 60:
                HandleStage60();
                break;
            case 70:
                Bean_Place_Switch(&bean[1], 80, 71);
                break;
            case 71:
                HandleStage71Or80(80, -300.0f, 1U);
                break;
            case 80:
                HandleStage71Or80(90, -580.0f, 0U);
                break;
            case 90:
                HandleStage90();
                break;
            case 100:
                HandleStage100();
                break;
            case 101:
                HandleStage101();
                break;
            case 110:
                Bean_Place_Switch(&bean[0], 800, 710);
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
