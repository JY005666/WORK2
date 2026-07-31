#include "UpperStateHelpers.h"

#include <math.h>
#include <stdlib.h>

static uint8_t s_final_turn_locked = 0U;

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
    return (fabsf(lidar.distance_aver - par.target_distance) < distance_tol) &&
           (fabsf(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < chassis_tol);
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
        if (fabsf(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < chassis_tol) {
            stage_flag = next_stage;
            ResetDistanceAndChassisMotors();
        }
    }
}

void ResetStage30PlacementState(void)
{
    s_final_turn_locked = 0U;
}

/*
 * 绗簩棰楄眴鐨勭粺涓€鎺ヨ繎鐘舵€侊細
 * 鏀惧畬绗竴涓眴瀛愬悗锛屽す鐖笂鍗囷紝鐒跺悗鏍规嵁绗竴涓眴瀛愮殑鐩爣绠卞瓙鍦ㄥ乏鍗婅竟杩樻槸闈炲乏鍗婅竟锛?
 * 鐩存帴缁欏乏璞嗘垨鍙宠眴瀵瑰簲鐨勫簳鐩樿窛绂诲拰浜戝彴瑙掑害銆?
 * 褰撹窛绂诲拰浜戝彴閮藉埌浣嶅悗锛岃繘鍏ユ姄鍙栫姸鎬併€?
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
    float final_chassis_target = 0.0f;

    if (target_box == NULL) {
        s_final_turn_locked = 0U;
        return;
    }

    Claw_degree_set(target_box->claw_angle, CLAW_UP);

    par.target_distance = target_box->distance;
    if (target_is_left_side) {
        final_chassis_target = GetFinalChassisForStage30(target_box, bean[2].target_position, 1U);
    } else {
        final_chassis_target = GetFinalChassisForStage30(target_box, bean[2].target_position, 0U);
    }

    if (!s_final_turn_locked &&
        lidar.distance_aver > SAFE_DIST_FOR_BOX_FINAL_TURN_MM) {
        s_final_turn_locked = 1U;
    }

    if (!s_final_turn_locked){ //
        if (target_is_left_side){par.degree_chassis = FIRST_BEAN_LEFT_SAFE_CCW_DEG;}
        else {par.degree_chassis = FIRST_BEAN_RIGHT_SAFE_CW_DEG;}
    } else { //
        par.degree_chassis = final_chassis_target;
        // if(bean[2].target_position != RIGHT_2&&bean[2].target_position != LEFT_2){par.degree_claw = DELIVERY_RELEASE_LIFT_TARGET_DEG;}
    }

    if(s_final_turn_locked){
        if ((fabsf(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < 8.0f) &&
            (fabsf(lidar.distance_aver - par.target_distance) < 15.0f)) {
            par.degree_claw = DELIVERY_RELEASE_LIFT_TARGET_DEG;
            if (hDJI[3].AxisData.AxisAngle_inDegree  > DELIVERY_RELEASE_LIFT_READY_DEG) {
                Motor_State_Reset(&hDJI[0]);
                Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                osDelay(200);
                Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                osDelay(200);
                Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                osDelay(200);
                Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                osDelay(200);
                Motor_State_Reset(&hDJI[2]);
                s_final_turn_locked = 0U;
                stage_flag = 31;
            }
        }
    }
}
