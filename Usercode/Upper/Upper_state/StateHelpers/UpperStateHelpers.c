#include "UpperStateHelpers.h"

#include <stdlib.h>

static uint8_t IsLeftSideBox(BoxPosition position)
{
    return (uint8_t)((position == LEFT_2) || (position == LEFT_1));
}

static Angle *GetBoxAngleByPosition(BoxPosition position)
{
    switch (position) {
        case LEFT_2:   return &box_left_2;
        case LEFT_1:   return &box_left_1;
        case MIDDLE_0: return &box_middle_0;
        case RIGHT_1:  return &box_right_1;
        case RIGHT_2:  return &box_right_2;
        default:       return NULL;
    }
}







static float GetFinalChassisForStage30(const Angle *target_box, BoxPosition position, uint8_t approached_from_left_side)
{
    if (target_box == NULL) {
        return 0.0f;
    }

    if (position != MIDDLE_0) {
        return target_box->chassis;
    }

    if (approached_from_left_side) {
        return box_middle_0_chassis_cw;
    }

    return box_middle_0_chassis_ccw;
}




uint8_t IsDistanceAndChassisReady(float distance_tol, float chassis_tol)
{
    return (abs(lidar.distance_aver - par.target_distance) < distance_tol) &&
           (abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < chassis_tol);
}

void ResetDistanceAndChassisMotors(void)
{
    Motor_State_Reset(&hDJI[0]);
    Motor_State_Reset(&hDJI[2]);
}

void CloseClawAndAdvance(uint16_t next_stage)
{
    Claw_degree_set(CLAW_CLOSE, CLAW_DOWN);
    osDelay(800);
    stage_flag = next_stage;
}

void LiftAndRotateToPlacement(float lift_target, float lift_ready_threshold, float chassis_target, float chassis_tol, uint16_t next_stage)
{
    par.degree_claw = lift_target;
    if (hDJI[3].AxisData.AxisAngle_inDegree < lift_ready_threshold) {
        par.degree_chassis = chassis_target;
        if (abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < chassis_tol) {
            stage_flag = next_stage;
            ResetDistanceAndChassisMotors();
        }
    }
}

/*
 * 第二颗豆的统一接近状态：
 * 放完第一个豆子后，夹爪上升，然后根据第一个豆子的目标箱子在左半边还是非左半边，
 * 直接给左豆或右豆对应的底盘距离和云台角度。
 * 当距离和云台都到位后，进入抓取状态。
 */
void HandleStage31_SecondBeanPickup(void)
{
    par.degree_claw = -600.0f;
    if (IsLeftSideBox(bean[2].target_position)) {
        g_second_bean_position = RIGHT;
        g_third_bean_position = LEFT;
        Claw_degree_set(bean_right.claw_angle, CLAW_UP);
    } else {
        g_second_bean_position = LEFT;
        g_third_bean_position = RIGHT;
        Claw_degree_set(bean_left.claw_angle, CLAW_UP);
    }

    Claw_degree_set(CLAW_OPEN, CLAW_DOWN);
    if(g_second_bean_position == RIGHT){
        par.degree_chassis = bean_right.chassis;
        par.target_distance = bean_right.distance;
    }
    else if(g_second_bean_position == LEFT){
        par.degree_chassis = bean_left.chassis;
        par.target_distance = bean_left.distance;
    }
        stage_flag = 40;
}

void HandleStage30_FirstBeanPlacement(void)
{
    Angle *target_box = GetBoxAngleByPosition(bean[2].target_position);
    uint8_t target_is_left_side = IsLeftSideBox(bean[2].target_position);

    if (target_box == NULL) {
        return;
    }

    Claw_degree_set(target_box->claw_angle, CLAW_UP);

    par.target_distance = target_box->distance;
    if (lidar.distance_aver <= SAFE_DIST_FOR_BOX_FINAL_TURN_MM){ //小于安全距离阈值
        if (target_is_left_side){par.degree_chassis = FIRST_BEAN_LEFT_SAFE_CCW_DEG;}
        else {par.degree_chassis = FIRST_BEAN_RIGHT_SAFE_CW_DEG;}
    } else if (lidar.distance_aver > SAFE_DIST_FOR_BOX_FINAL_TURN_MM) { //大于安全距离阈值
        if (target_is_left_side) {par.degree_chassis = GetFinalChassisForStage30(target_box, bean[2].target_position, 1U);} 
        else {par.degree_chassis =  GetFinalChassisForStage30(target_box, bean[2].target_position, 0U);}
    }
    /* 先完成底盘与云台对位，再执行放置动作。 */
    if(lidar.distance_aver > SAFE_DIST_FOR_BOX_FINAL_TURN_MM){
        if ((abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < 8.0f) &&
            (abs(lidar.distance_aver - par.target_distance) < 8.0f)) {
            par.degree_claw = -280.0f;
            if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -290.0f) {
                Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                osDelay(500);
                ResetDistanceAndChassisMotors();
                stage_flag = 31;
            }
        }
    }

}
