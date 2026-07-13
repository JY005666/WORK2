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

static Bean *GetActiveBean(void)
{
    if (g_active_bean_state == ACTIVE_BEAN_LEFT) {
        return &bean[1];
    }
    if (g_active_bean_state == ACTIVE_BEAN_RIGHT) {
        return &bean[0];
    }
    if (g_active_bean_state == ACTIVE_BEAN_MIDDLE) {
        return &bean[2];
    }
    return NULL;
}

static uint8_t IsRightSideBox(BoxPosition position)
{
    return (uint8_t)((position == RIGHT_1) || (position == RIGHT_2));
}

uint8_t ApplyDeliveryLiftGate(BeanPosition bean_position)
{
    float lift_target = MIDDLE_BEAN_DELIVERY_LIFT_TARGET_DEG;
    float lift_ready = MIDDLE_BEAN_DELIVERY_LIFT_READY_DEG;

    if (bean_position == LEFT) {
        lift_target = LEFT_BEAN_DELIVERY_LIFT_TARGET_DEG;
        lift_ready = LEFT_BEAN_DELIVERY_LIFT_READY_DEG;
    } else if (bean_position == RIGHT) {
        lift_target = RIGHT_BEAN_DELIVERY_LIFT_TARGET_DEG;
        lift_ready = RIGHT_BEAN_DELIVERY_LIFT_READY_DEG;
    }

    par.degree_claw = lift_target;
    return (uint8_t)(hDJI[3].AxisData.AxisAngle_inDegree <= lift_ready);
}

static void FinishBeanPlacement(Bean *b, uint16_t next_stage)
{
    if (b != NULL) {
        g_last_placed_box_position = b->target_position;
    }

    Claw_degree_set(CLAW_HALF_OPEN, CLAW_DOWN);
    osDelay(500);
    stage_flag = next_stage;
}

static void TryPlaceBean(Bean *b,
                         const Angle *target_box,
                         float arm_target,
                         float arm_ready_margin,
                         float chassis_tol,
                         float distance_tol,
                         uint16_t next_stage)
{
    float placement_chassis = 0.0f;

    if (b == NULL || target_box == NULL) {
        return;
    }

    if ((b->target_position == MIDDLE_0) && (b->position == LEFT)) {
        placement_chassis = box_middle_0_chassis_cw;
    } else if ((b->target_position == MIDDLE_0) && (b->position == RIGHT)) {
        placement_chassis = box_middle_0_chassis_ccw;
    } else {
        placement_chassis = target_box->chassis;
    }

    Claw_degree_set(target_box->claw_angle, CLAW_UP);
    par.target_distance = target_box->distance;
    par.degree_chassis = placement_chassis;

    if ((abs(hDJI[2].AxisData.AxisAngle_inDegree - par.degree_chassis) < chassis_tol) &&
        (abs(lidar.distance_aver - par.target_distance) < distance_tol)) {
        par.degree_claw = arm_target;
        if (hDJI[3].AxisData.AxisAngle_inDegree - par.degree_claw > arm_ready_margin) {
            FinishBeanPlacement(b, next_stage);
        }
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

static float GetFinalChassisForActiveBeanDelivery(const Angle *target_box, Bean *active_bean)
{
    if (target_box == NULL || active_bean == NULL) {
        return 0.0f;
    }

    if (active_bean->target_position != MIDDLE_0) {
        return target_box->chassis;
    }

    if (active_bean->position == LEFT) {
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

/*
 * 第二颗豆的统一接近状态：
 * 放完第一个豆子后，夹爪上升，然后根据第一个豆子的目标箱子在左半边还是非左半边，
 * 直接给左豆或右豆对应的底盘距离和云台角度。
 * 当距离和云台都到位后，进入抓取状态。
 */
void HandleStage31_SecondBeanPickup(void)
{
    const Angle *next_target = NULL;
    uint16_t next_stage = 0;


    /*
     * 豆子和箱子的左右是相对关系：
     * - 如果第一个豆子的目标箱子在左半边，下一步去抓右豆
     * - 如果第一个豆子的目标箱子在非左半边，下一步去抓左豆
     */
    if (IsLeftSideBox(bean[2].target_position)) {
        next_target = &bean_right;
        g_active_bean_state = ACTIVE_BEAN_RIGHT;
        g_third_bean_position = LEFT;
    } else {
        next_target = &bean_left;
        g_active_bean_state = ACTIVE_BEAN_LEFT;
        g_third_bean_position = RIGHT;
    }
    next_stage = 50;

    if (next_target == NULL) {
        return;
    }

    Claw_degree_set(CLAW_OPEN, CLAW_DOWN);
    Claw_degree_set(next_target->claw_angle, CLAW_UP);
    SetApproachTarget(next_target);

    if (IsDistanceAndChassisReady(5.0f, 2.0f)) {
        stage_flag = next_stage;
    }
}

/*
 * 第二个豆子抓取完成后的送箱逻辑：
 * 1. 左豆：
 *    - 目标箱子在非右边：先走目标距离，距离过线后再给最终云台角
 *    - 目标箱子在右边：先给逆时针避障角，等云台角度大于 0 后再走目标距离，
 *      距离过线后再给最终云台角
 * 2. 右豆：
 *    - 目标箱子在非左边：先走目标距离，距离过线后再给最终云台角
 *    - 目标箱子在左边：先给顺时针避障角，等云台角度小于 0 后再走目标距离，
 *      距离过线后再给最终云台角
 */
void HandleActiveBeanDelivery(uint16_t next_stage_left, uint16_t next_stage_right)
{
    Bean *active_bean = GetActiveBean();
    Angle *target_box = NULL;
    uint16_t next_stage = 0;
    uint8_t need_avoid = 0U;
    uint8_t used_ccw_avoid = 0U;
    float avoid_chassis_target = 0.0f;


    /*
     * 抓完第二/第三颗豆子后，必须先把升降轴抬到安全高度，
     * 再允许底盘和云台进入送箱过程，避免“还没抬起来就开始走”。
     */

    if (active_bean == NULL) {
        return;
    }

    if (!ApplyDeliveryLiftGate(active_bean->position)) {
        return;
    }

    target_box = GetBoxAngleByPosition(active_bean->target_position);
    if (target_box == NULL) {
        return;
    }

    Claw_degree_set(target_box->claw_angle, CLAW_UP);

    if (g_active_bean_state == ACTIVE_BEAN_LEFT) {
        next_stage = next_stage_left;
        if (IsRightSideBox(active_bean->target_position)) {
            need_avoid = 1U;
            used_ccw_avoid = 1U;
            avoid_chassis_target = SECOND_BEAN_LEFT_TO_RIGHT_AVOID_CCW_DEG;
        }
    } else if (g_active_bean_state == ACTIVE_BEAN_RIGHT) {
        next_stage = next_stage_right;
        if (IsLeftSideBox(active_bean->target_position)) {
            need_avoid = 1U;
            avoid_chassis_target = SECOND_BEAN_RIGHT_TO_LEFT_AVOID_CW_DEG;
        }
    } else {
        return;
    }

    if (!g_delivery_distance_enabled) {
        if (need_avoid) {
            par.degree_chassis = avoid_chassis_target;
            if ((used_ccw_avoid && (hDJI[2].AxisData.AxisAngle_inDegree > 0.0f)) ||
                (!used_ccw_avoid && (hDJI[2].AxisData.AxisAngle_inDegree < 0.0f))) {
                g_delivery_distance_enabled = 1U;
            }
        } else {
            g_delivery_distance_enabled = 1U;
        }
    }

    if (g_delivery_distance_enabled) {
        par.target_distance = target_box->distance;
        if (!g_delivery_final_turn_enabled && (lidar.distance_aver > SECOND_BEAN_FINAL_TURN_MM)) {
            g_delivery_final_turn_enabled = 1U;
        }
    }

    if (g_delivery_final_turn_enabled) {
        par.degree_chassis = GetFinalChassisForActiveBeanDelivery(target_box, active_bean);
    } else if (need_avoid) {
        par.degree_chassis = avoid_chassis_target;
    }

    if (IsDistanceAndChassisReady(5.0f, 8.0f)) {
        stage_flag = next_stage;
    }
}


/*
 * 第三颗豆抓取的可试版逻辑：
 * 直接取“剩下的最后一颗豆”，把它的底盘距离和云台角度目标赋给程序。
 * 当前版本先不加复杂避障，只验证第三颗豆能否被正确接近并抓取。
 */
void HandleStage900_ThirdBeanPickup(void)
{
    const Angle *last_target = NULL;

    par.degree_claw = -650.0f;

    /*
     * 第三颗豆子在第二颗豆子的抓取路线确定时就已经记录下来，
     * 这里直接按记录值取目标，避免 g_active_bean_state 被后续状态覆盖后反推失败。
     */
    if (g_third_bean_position == LEFT) {
        last_target = &bean_left;
        g_active_bean_state = ACTIVE_BEAN_LEFT;
    } else if (g_third_bean_position == RIGHT) {
        last_target = &bean_right;
        g_active_bean_state = ACTIVE_BEAN_RIGHT;
    } else if (g_third_bean_position == MIDDLE) {
        last_target = &bean_middle;
        g_active_bean_state = ACTIVE_BEAN_MIDDLE;
    } else {
        return;
    }

    if (last_target == NULL) {
        return;
    }

    Claw_degree_set(CLAW_OPEN, CLAW_DOWN);
    Claw_degree_set(last_target->claw_angle, CLAW_UP);
    SetApproachTarget(last_target);

    if (IsDistanceAndChassisReady(5.0f, 2.0f)) {
        stage_flag = 910;
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

void Bean_Place_Switch(Bean *b, uint16_t next_stage)
{
    if (b == NULL) {
        return;
    }

    switch (b->target_position) {
        case LEFT_2: {
            TryPlaceBean(b, &box_left_2, -170.0f, -230.0f, 8.0f, 5.0f, next_stage);
        } break;
        case LEFT_1: {
            TryPlaceBean(b, &box_left_1, -170.0f, -230.0f, 8.0f, 5.0f, next_stage);
        } break;
        case MIDDLE_0: {
            TryPlaceBean(b, &box_middle_0, -170.0f, -230.0f, 8.0f, 5.0f, next_stage);
        } break;
        case RIGHT_1: {
            TryPlaceBean(b, &box_right_1, -170.0f, -200.0f, 8.0f, 5.0f, next_stage);
        } break;
        case RIGHT_2: {
            TryPlaceBean(b, &box_right_2, -190.0f, -230.0f, 8.0f, 5.0f, next_stage);
        } break;
    }
}
