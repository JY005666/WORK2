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

void SetApproachTarget(const Angle *target)
{
    par.degree_chassis = target->chassis;
    par.target_distance = target->distance;
}

void PrepareBeanPickup(const Angle *target, float claw_hold_degree, float arm_ready_threshold)
{
    par.degree_claw = claw_hold_degree;
    if (hDJI[3].AxisData.AxisAngle_inDegree < arm_ready_threshold) {
        Claw_degree_set(CLAW_OPEN, CLAW_DOWN);
        Claw_degree_set(target->claw_angle, CLAW_UP);
        SetApproachTarget(target);
    }
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

void HandleStage31Or40(uint16_t next_stage, float arm_ready_threshold, uint8_t require_distance_gate)
{
    PrepareBeanPickup(&bean_left, -650.0f, arm_ready_threshold);
    if (require_distance_gate && lidar.distance_aver >= 2000.0f) {
        return;
    }
    if (IsDistanceAndChassisReady(5.0f, 2.0f)) {
        stage_flag = next_stage;
    }
}

void HandleStage71Or80(uint16_t next_stage, float arm_ready_threshold, uint8_t require_distance_gate)
{
    PrepareBeanPickup(&bean_right, -650.0f, arm_ready_threshold);
    if (require_distance_gate && lidar.distance_aver >= 2000.0f) {
        return;
    }
    if (IsDistanceAndChassisReady(5.0f, 2.0f)) {
        stage_flag = next_stage;
    }
}

/*
 * 第二颗豆的统一接近状态：
 * 不再区分“先进入左豆入口状态”还是“先进入右豆入口状态”，
 * 而是在这里直接根据第一个豆子的目标箱子位置，决定下一步去抓左豆还是右豆。
 */
void HandleStage31_SecondBeanPickup(void)
{
    if (IsLeftSideBox(bean[2].target_position)) {
        HandleStage31Or40(50, -450.0f, 1U);
    } else {
        HandleStage71Or80(90, -580.0f, 0U);
    }
}

/*
 * 第一个豆子（中间豆）抓取完成后的放箱逻辑：
 * 1. 先根据目标箱子位于左半边还是非左半边，给一个“临时避障角”
 * 2. 与此同时底盘先直接朝目标箱子的 distance 前进
 * 3. 当距离已经大于安全阈值后，再把云台切换到该箱子的最终放置角
 *
 * 大多数箱子最终角度统一使用 target_box->chassis。
 * 只有中间箱位 MIDDLE_0 单独使用 box_middle_0_chassis_cw / box_middle_0_chassis_ccw。
 */
void HandleStage30_FirstBeanPlacement(void)
{
    Angle *target_box = GetBoxAngleByPosition(bean[2].target_position);
    uint8_t target_is_left_side = IsLeftSideBox(bean[2].target_position);

    if (target_box == NULL) {
        return;
    }

    Claw_degree_set(target_box->claw_angle, CLAW_UP);
    par.target_distance = target_box->distance;

    if (target_is_left_side) {
        /* 左半边箱子：先逆时针绕开障碍，距离走够以后再切到箱子的最终角 */
        par.degree_chassis = FIRST_BEAN_LEFT_SAFE_CCW_DEG;
        if (lidar.distance_aver > SAFE_DIST_FOR_BOX_FINAL_TURN_MM) {
            par.degree_chassis = GetFinalChassisForStage30(target_box, bean[2].target_position, 1U);
        }
    } else {
        /* 非左半边箱子：先顺时针绕开障碍，距离走够以后再切到箱子的最终角 */
        par.degree_chassis = FIRST_BEAN_RIGHT_SAFE_CW_DEG;
        if (lidar.distance_aver > SAFE_DIST_FOR_BOX_FINAL_TURN_MM) {
            par.degree_chassis = GetFinalChassisForStage30(target_box, bean[2].target_position, 0U);
        }
    }

    /* 先完成底盘与云台对位，再执行放置动作。 */
    if ((abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < 8.0f) &&
        (abs(lidar.distance_aver - par.target_distance) < 5.0f)) {
        par.degree_claw = -170.0f;
        if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -230.0f) {
            Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
            osDelay(500);
            Motor_State_Reset(&hDJI[0]);
            Motor_State_Reset(&hDJI[2]);
            /*
             * 第一个豆子放完后的下一目标，不再看它原本是“中间豆”，
             * 而是看它被放到了左半边还是非左半边：
             * - 左半边箱子：下一步去抓左豆
             * - 非左半边箱子：下一步去抓右豆
             */
            stage_flag = 31;
        }
    }
}

void Bean_Place_Switch(Bean *b, uint16_t next_normal, uint16_t next_special)
{
    switch (b->target_position) {
        case LEFT_2: {
            Claw_degree_set(box_left_2.claw_angle, CLAW_UP);
            par.target_distance = box_left_2.distance;
            par.degree_chassis = box_left_2.chassis;
            if ((abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < 8.0f) &&
                (abs(lidar.distance_aver - par.target_distance) < 5.0f)) {
                par.degree_claw = -170.0f;
                if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -230.0f) {
                    Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                    osDelay(500);
                    Motor_State_Reset(&hDJI[0]);
                    Motor_State_Reset(&hDJI[2]);
                    if (b->position == MIDDLE) {
                        stage_flag = next_special;
                    } else {
                        stage_flag = next_normal;
                    }
                }
            }
        } break;
        case LEFT_1: {
            Claw_degree_set(box_left_1.claw_angle, CLAW_UP);
            par.target_distance = box_left_1.distance;
            par.degree_chassis = box_left_1.chassis;
            if ((abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < 8.0f) &&
                (abs(lidar.distance_aver - par.target_distance) < 5.0f)) {
                par.degree_claw = -170.0f;
                if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -230.0f) {
                    Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                    osDelay(500);
                    Motor_State_Reset(&hDJI[0]);
                    Motor_State_Reset(&hDJI[2]);
                    stage_flag = next_normal;
                }
            }
        } break;
        case MIDDLE_0: {
            Claw_degree_set(box_middle_0.claw_angle, CLAW_UP);
            par.target_distance = box_middle_0.distance;
            if (lidar.distance_aver > 1800.0f) {
                par.degree_chassis = box_middle_0.chassis;
                if ((abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < 8.0f) &&
                    (abs(lidar.distance_aver - par.target_distance) < 5.0f)) {
                    par.degree_claw = -190.0f;
                    if (hDJI[3].AxisData.AxisAngle_inDegree > -230.0f) {
                        Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                        osDelay(500);
                        Motor_State_Reset(&hDJI[0]);
                        Motor_State_Reset(&hDJI[2]);
                        stage_flag = next_normal;
                    }
                }
            }
        } break;
        case RIGHT_1: {
            Claw_degree_set(box_right_1.claw_angle, CLAW_UP);
            par.target_distance = box_right_1.distance;
            if (lidar.distance_aver > 1800.0f) {
                par.degree_chassis = box_right_1.chassis;
                if ((abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < 8.0f) &&
                    (abs(lidar.distance_aver - par.target_distance) < 5.0f)) {
                    par.degree_claw = -170.0f;
                    if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -200.0f) {
                        Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                        osDelay(500);
                        Motor_State_Reset(&hDJI[0]);
                        Motor_State_Reset(&hDJI[2]);
                        stage_flag = next_normal;
                    }
                }
            }
        } break;
        case RIGHT_2: {
            Claw_degree_set(box_right_2.claw_angle, CLAW_UP);
            if (lidar.distance_aver < 2000.0f) {
                par.target_distance = 2155.0f;
            }
            if (lidar.distance_aver > 2000.0f) {
                par.degree_chassis = box_right_2.chassis;
                if (abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < 20.0f) {
                    par.target_distance = box_right_2.distance;
                    if (abs(lidar.distance_aver - par.target_distance) < 5.0f) {
                        par.degree_claw = -190.0f;
                        if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > -230.0f) {
                            Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
                            osDelay(500);
                            Motor_State_Reset(&hDJI[0]);
                            Motor_State_Reset(&hDJI[2]);
                            if (b->position == LEFT) {
                                stage_flag = next_special;
                            } else {
                                stage_flag = next_normal;
                            }
                        }
                    }
                }
            }
        } break;
    }
}
