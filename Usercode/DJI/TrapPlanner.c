/*============================================================
 *  梯形速度规划器 —— 速度前馈 + 位置修正
 *============================================================*/

#include "TrapPlanner.h"
#include "math.h"

void TrapPlanner_Start(TrapPlanner_t *plan, float target_pos) {
    if (plan == NULL) return;

    plan->start_pos   = (float)distance_offset - lidar.distance_aver;
    plan->target_pos  = target_pos;
    plan->total_dist  = target_pos - plan->start_pos;
    plan->start_tick  = HAL_GetTick();
    plan->arrival_cnt = 0;
    plan->active      = 1;

    float dist_abs = fabsf(plan->total_dist);
    if (dist_abs < TRAP_STOP_THRESHOLD_MM) { plan->active = 0; return; }

    float t_acc_min = plan->max_vel / plan->acc;
    float s_acc = 0.5f * plan->acc * t_acc_min * t_acc_min;

    if (2.0f * s_acc >= dist_abs) {
        t_acc_min = sqrtf(dist_abs / plan->acc);
        plan->t_acc = t_acc_min; plan->t_const = 0.0f;
    } else {
        plan->t_acc = t_acc_min;
        plan->t_const = (dist_abs - 2.0f * s_acc) / plan->max_vel;
    }
    plan->t_total = 2.0f * plan->t_acc + plan->t_const;
}

float TrapPlanner_GetTargetPos(TrapPlanner_t *plan) {
    if (plan == NULL || !plan->active) return plan->target_pos;

    float dt = (float)(HAL_GetTick() - plan->start_tick) / 1000.0f;
    float dir = (plan->total_dist >= 0.0f) ? 1.0f : -1.0f;
    float dist_abs = fabsf(plan->total_dist);
    float s = 0.0f;

    if (dt <= 0.0f) s = 0.0f;
    else if (dt < plan->t_acc) s = 0.5f * plan->acc * dt * dt;
    else if (dt < plan->t_acc + plan->t_const) {
        float v_max = plan->acc * plan->t_acc;
        s = 0.5f * plan->acc * plan->t_acc * plan->t_acc;
        s += v_max * (dt - plan->t_acc);
    } else if (dt < plan->t_total) {
        float t_remain = plan->t_total - dt;
        s = dist_abs - 0.5f * plan->acc * t_remain * t_remain;
    } else { s = dist_abs; plan->active = 0; }

    if (s > dist_abs) s = dist_abs;
    if (s < 0.0f) s = 0.0f;
    return plan->start_pos + dir * s;
}

/**
 * 速度前馈：返回当前时刻的期望速度 (mm/s)
 * 方向和 TrapPlanner_GetTargetPos 的方向一致
 */
float TrapPlanner_GetTargetVel(TrapPlanner_t *plan) {
    if (plan == NULL || !plan->active) return 0.0f;

    float dt = (float)(HAL_GetTick() - plan->start_tick) / 1000.0f;
    float dir = (plan->total_dist >= 0.0f) ? 1.0f : -1.0f;
    float v = 0.0f;

    if (dt <= 0.0f) v = 0.0f;
    else if (dt < plan->t_acc)
        v = plan->acc * dt;                     // 加速段：v = a*t
    else if (dt < plan->t_acc + plan->t_const)
        v = plan->acc * plan->t_acc;            // 匀速段：v = v_max
    else if (dt < plan->t_total) {
        float t_remain = plan->t_total - dt;
        v = plan->acc * t_remain;               // 减速段：v = a*t_remain
    } else v = 0.0f;

    return dir * v;
}

uint8_t TrapPlanner_IsArrived(TrapPlanner_t *plan, float current_pos) {
    if (plan == NULL || !plan->active) return 1;
    float err = plan->target_pos - current_pos;
    if (fabsf(err) < TRAP_STOP_THRESHOLD_MM) {
        plan->arrival_cnt++;
        if (plan->arrival_cnt >= TRAP_ARRIVAL_CNT_THR) { plan->active = 0; return 1; }
    } else plan->arrival_cnt = 0;
    return 0;
}

/**
 * 速度前馈 + 位置小比例修正 的距离伺服
 */
void Distance_servo_WithPlanning(TrapPlanner_t *plan, DJI_t *motor1)
{
    if (plan == NULL || motor1 == NULL)
        return;

    // =========================================================
    // 1. 获取当前位置
    // =========================================================
    float current_pos =
        (float)distance_offset - lidar.distance_aver;

    // =========================================================
    // 2. 获取规划目标
    // =========================================================
    float target_pos =
        TrapPlanner_GetTargetPos(plan);

    float target_vel =
        TrapPlanner_GetTargetVel(plan);

    // =========================================================
    // 3. 到达检测
    // =========================================================
    if (TrapPlanner_IsArrived(plan, current_pos))
    {
        // 清除PID状态
        PID_Clear(&motor1->speedPID);
        PID_Clear(&motor1->posPID);

        CanTransmit_DJI_1234(&hcan1, 0, 0, 0, 0);

        return;
    }

    // =========================================================
    // 4. 位置误差修正（小比例）
    // =========================================================
    float pos_error =
        target_pos - current_pos;

    // 小位置修正
    // 这里只负责“追回误差”
    float pos_correction =
        3.0f * pos_error;

    // =========================================================
    // 5. 合成目标速度
    // =========================================================
    float speed_ref =
        target_vel + pos_correction;

    // =========================================================
    // 6. 限速
    // =========================================================
    if (speed_ref > 500.0f)
        speed_ref = 500.0f;

    if (speed_ref < -500.0f)
        speed_ref = -500.0f;

    // =========================================================
    // 7. 接近目标时停止
    // =========================================================
    if (fabsf(pos_error) < 0.5f)
    {
        speed_ref = 0.0f;
    }

    // =========================================================
    // 8. 速度闭环 PID
    // =========================================================

    // 目标速度
    motor1->speedPID.ref =
        speed_ref;

    // 实际速度（编码器反馈）
    motor1->speedPID.fdb =
        motor1->FdbData.rpm;

    // PID计算
    PID_Calc(&motor1->speedPID);

    // PID输出作为扭矩
    int torque =
        (int)(motor1->speedPID.output);

    // =========================================================
    // 9. 扭矩限幅
    // =========================================================
    if (torque > 3000)
        torque = 3000;

    if (torque < -3000)
        torque = -3000;

    // =========================================================
    // 10. 输出到电机
    // =========================================================
    CanTransmit_DJI_1234(
        &hcan1,
        -torque,
        torque,
        0,
        0);
}
