#include "Caculate.h"

#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

void PID_Calc(PID_t *pid)
{
    if (pid == NULL) return;

    pid->cur_error = pid->ref - pid->fdb;
    pid->output += pid->KP * (pid->cur_error - pid->error[1]) +
                   pid->KI * pid->cur_error +
                   pid->KD * (pid->cur_error - 2.0f * pid->error[1] + pid->error[0]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->ref - pid->fdb;

    if (pid->output > pid->outputMax) pid->output = pid->outputMax;
    if (pid->output < -pid->outputMax) pid->output = -pid->outputMax;

    if (pid->outputMax > 0.0f &&
        pid->outputMin > 0.0f &&
        fabsf(pid->output) < pid->outputMin) {
        pid->output = 0.0f;
    }
}

void P_Calc(PID_t *pid)
{
    if (pid == NULL) return;

    pid->cur_error = pid->ref - pid->fdb;
    pid->output = pid->KP * pid->cur_error;

    if (pid->output > pid->outputMax) pid->output = pid->outputMax;
    if (pid->output < -pid->outputMax) pid->output = -pid->outputMax;

    if (pid->outputMin > 0.0f && fabsf(pid->output) < pid->outputMin) {
        pid->output = 0.0f;
    }
}

void positionServo(float ref, DJI_t *motor)
{
    if (motor == NULL) return;

    motor->posPID.ref = ref;
    motor->posPID.fdb = motor->AxisData.AxisAngle_inDegree;
    PID_Calc(&motor->posPID);

    motor->speedPID.ref = motor->posPID.output;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
}

void speedServo(float ref, DJI_t *motor)
{
    if (motor == NULL) return;

    motor->speedPID.ref = ref;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
}

typedef struct {
    DJI_t *motor;
    uint8_t initialized;
    uint8_t arrived;
    uint8_t arrived_confirmed;
    uint8_t use_reliable_near_target;
    float target_distance;
    float last_speed_ref;
    float last_error;
    uint32_t last_tick;
    uint8_t kick_start_active;
    uint32_t kick_start_start_tick;
    float filtered_distance;
    uint32_t arrived_tick;
} DistanceServoPlanner_t;

static DistanceServoPlanner_t distance_planner = {0};
DistanceServoDebug_t g_distance_servo_debug = {0};
static float s_distance_last_valid_distance = 0.0f;
static uint32_t s_distance_last_valid_tick = 0U;
static float s_distance_jump_candidate_distance = 0.0f;
static uint8_t s_distance_last_valid_ready = 0U;
static uint8_t s_distance_jump_candidate_valid = 0U;
static float s_distance_last_raw_distance = 0.0f;
static uint32_t s_distance_last_raw_tick = 0U;
static uint32_t s_distance_jump_candidate_tick = 0U;
static uint8_t s_distance_last_raw_ready = 0U;

static float Limit_Float(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float Abs_Limit_Float(float value, float abs_max)
{
    if (abs_max < 0.0f) abs_max = -abs_max;
    if (value > abs_max) return abs_max;
    if (value < -abs_max) return -abs_max;
    return value;
}

static float Sign_Float(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

static uint8_t Float_Is_Usable(float value)
{
    return (uint8_t)((value == value) && (value > -1.0e30f) && (value < 1.0e30f));
}

static uint8_t Distance_Is_Valid(float distance)
{
    if (!Float_Is_Usable(distance)) return 0U;
    if (distance < DIST_SERVO_SENSOR_MIN_MM) return 0U;
    if (distance > DIST_SERVO_SENSOR_MAX_MM) return 0U;
    return 1U;
}

static float DistanceServo_GetDt(uint32_t now_tick)
{
    uint32_t dt_ms = 0U;
    float dt = 0.0f;

    if (!distance_planner.initialized || distance_planner.last_tick == 0U) {
        return DIST_SERVO_DEFAULT_DT_S;
    }

    dt_ms = now_tick - distance_planner.last_tick;
    if (dt_ms == 0U) {
        return DIST_SERVO_DEFAULT_DT_S;
    }

    dt = (float)dt_ms * 0.001f;
    if (dt > DIST_SERVO_MAX_DT_S) dt = DIST_SERVO_MAX_DT_S;
    if (dt < DIST_SERVO_DEFAULT_DT_S) dt = DIST_SERVO_DEFAULT_DT_S;
    return dt;
}

static float LowPass_Filter(float raw, float *filtered, float alpha)
{
    *filtered = alpha * raw + (1.0f - alpha) * (*filtered);
    return *filtered;
}

/*
 * 距离样本筛选逻辑：
 * 1. 原始测距必须先落在 200~2600mm 的有效工作区间内。
 * 2. 如果新测距相对“上一次有效测距”的变化速度超过物理上限，
 *    则先不让它进入伺服，而是暂存为候选值。
 * 3. 只有当这个候选值持续存在到“物理上已经来得及变化到那里”时，
 *    才把它接纳为新的有效测距。
 *
 * 这样可以同时滤掉：
 * - 超出赛程量程的无效值
 * - 突然跳到很远处的离谱值
 * - 连续两三帧重复出现的异常值
 */
static uint8_t DistanceServo_GetReliableSample(float raw_distance, uint32_t now_tick, float *reliable_distance)
{
    float sample_dt_s = DIST_SERVO_DEFAULT_DT_S;
    float max_allowed_delta = 0.0f;
    uint32_t candidate_elapsed_ms = 0U;
    float candidate_elapsed_s = 0.0f;
    uint32_t hold_ms = 0U;
    float prev_raw_distance = s_distance_last_raw_distance;
    uint8_t prev_raw_ready = s_distance_last_raw_ready;
    float candidate_direction = 0.0f;
    float raw_direction = 0.0f;
    float raw_step = 0.0f;
    float continuous_step_limit = DIST_SERVO_CONTINUOUS_CANDIDATE_STEP_MM;
    uint8_t allow_small_reverse_jitter = 0U;

    if (reliable_distance == NULL) {
        return 0U;
    }

    if (!Float_Is_Usable(raw_distance)) {
        if (s_distance_last_valid_ready) {
            hold_ms = now_tick - s_distance_last_valid_tick;
            if (hold_ms > DIST_SERVO_INVALID_HOLD_MS) {
                return 0U;
            }
            *reliable_distance = s_distance_last_valid_distance;
            return 1U;
        }
        return 0U;
    }

    /*
     * 串口线程有时会在多个控制周期里重复输出同一个距离值。
     * 小变化时认为还是同一帧数据，不重新计算“采样时间间隔”。
     */
    if (!s_distance_last_raw_ready ||
        fabsf(raw_distance - s_distance_last_raw_distance) >= DIST_SERVO_NEW_SAMPLE_EPS_MM) {
        if (s_distance_last_raw_ready && now_tick > s_distance_last_raw_tick) {
            sample_dt_s = (float)(now_tick - s_distance_last_raw_tick) * 0.001f;
            if (sample_dt_s > DIST_SERVO_MAX_SAMPLE_DT_S) {
                sample_dt_s = DIST_SERVO_MAX_SAMPLE_DT_S;
            }
            if (sample_dt_s < DIST_SERVO_DEFAULT_DT_S) {
                sample_dt_s = DIST_SERVO_DEFAULT_DT_S;
            }
        }

        s_distance_last_raw_distance = raw_distance;
        s_distance_last_raw_tick = now_tick;
        s_distance_last_raw_ready = 1U;
    } else {
        if ((now_tick - s_distance_last_raw_tick) > DIST_SERVO_RAW_STALE_MS) {
            return 0U;
        }
        /*
         * 如果重复输出的是同一个“跳变候选值”，不能直接返回，
         * 否则候选值永远无法按时间累计到“物理上合理”。
         * 这里允许它继续往后走，后面会用 candidate_elapsed_s 做斜率判定。
         */
        if (s_distance_jump_candidate_valid &&
            fabsf(raw_distance - s_distance_jump_candidate_distance) <= DIST_SERVO_CANDIDATE_MATCH_MM) {
            sample_dt_s = DIST_SERVO_DEFAULT_DT_S;
        } else {
            if (s_distance_last_valid_ready) {
                *reliable_distance = s_distance_last_valid_distance;
                return 1U;
            }

            *reliable_distance = raw_distance;
            return 1U;
        }
    }

    if (raw_distance < DIST_SERVO_EFFECTIVE_MIN_MM || raw_distance > DIST_SERVO_EFFECTIVE_MAX_MM) {
        if (s_distance_last_valid_ready) {
            hold_ms = now_tick - s_distance_last_valid_tick;
            if (hold_ms > DIST_SERVO_INVALID_HOLD_MS) {
                return 0U;
            }
            *reliable_distance = s_distance_last_valid_distance;
            return 1U;
        }
        return 0U;
    }

    if (!s_distance_last_valid_ready) {
        s_distance_last_valid_distance = raw_distance;
        s_distance_last_valid_tick = now_tick;
        s_distance_last_valid_ready = 1U;
        s_distance_jump_candidate_valid = 0U;
        *reliable_distance = raw_distance;
        return 1U;
    }

    max_allowed_delta = DIST_SERVO_MAX_VALID_SLOPE_MM_PER_S * sample_dt_s;
    if (raw_distance < s_distance_last_valid_distance) {
        /* 由远到近时放宽一点接纳速度，避免 reliable 长时间卡在远处。 */
        max_allowed_delta *= 1.6f;
    }
    if (fabsf(raw_distance - s_distance_last_valid_distance) <= max_allowed_delta) {
        s_distance_last_valid_distance = raw_distance;
        s_distance_last_valid_tick = now_tick;
        s_distance_jump_candidate_valid = 0U;
        *reliable_distance = raw_distance;
        return 1U;
    }

    if (!s_distance_jump_candidate_valid) {
        s_distance_jump_candidate_distance = raw_distance;
        s_distance_jump_candidate_tick = now_tick;
        s_distance_jump_candidate_valid = 1U;
        *reliable_distance = s_distance_last_valid_distance;
        return 1U;
    }

    if (fabsf(raw_distance - s_distance_jump_candidate_distance) > DIST_SERVO_CANDIDATE_MATCH_MM) {
        candidate_direction = Sign_Float(s_distance_jump_candidate_distance - s_distance_last_valid_distance);
        raw_direction = Sign_Float(raw_distance - s_distance_last_valid_distance);
        raw_step = prev_raw_ready ? fabsf(raw_distance - prev_raw_distance) : 0.0f;
        if (candidate_direction < 0.0f) {
            continuous_step_limit *= 1.5f;
            /*
             * 由远到近时，测距模块轻微晃动可能会让原始值短暂回摆一点点。
             * 只要它仍然比 last_valid 更近，就不要立刻把整段候选重新计时。
             */
            if (raw_direction > 0.0f &&
                raw_distance < s_distance_last_valid_distance &&
                prev_raw_ready &&
                raw_step <= (DIST_SERVO_CANDIDATE_MATCH_MM * 0.5f)) {
                allow_small_reverse_jitter = 1U;
            }
        }

        if (candidate_direction == 0.0f ||
            (!allow_small_reverse_jitter && raw_direction != candidate_direction) ||
            (prev_raw_ready && raw_step > continuous_step_limit)) {
            s_distance_jump_candidate_distance = raw_distance;
            s_distance_jump_candidate_tick = now_tick;
            *reliable_distance = s_distance_last_valid_distance;
            return 1U;
        }

        /*
         * 连续同方向快速接近时，不要因为每一步都超出 match 窗口就反复重置候选。
         * 这里更新候选距离，但保留原始 candidate_tick，让累计时间继续增长。
         */
        s_distance_jump_candidate_distance = raw_distance;
    }

    candidate_elapsed_ms = now_tick - s_distance_jump_candidate_tick;
    candidate_elapsed_s = (float)candidate_elapsed_ms * 0.001f;
    if (candidate_elapsed_s < DIST_SERVO_DEFAULT_DT_S) {
        candidate_elapsed_s = DIST_SERVO_DEFAULT_DT_S;
    }

    max_allowed_delta = DIST_SERVO_MAX_VALID_SLOPE_MM_PER_S * candidate_elapsed_s;
    if (raw_distance < s_distance_last_valid_distance) {
        max_allowed_delta *= 1.6f;
    }
    if (fabsf(raw_distance - s_distance_last_valid_distance) <= max_allowed_delta) {
        s_distance_last_valid_distance = raw_distance;
        s_distance_last_valid_tick = now_tick;
        s_distance_jump_candidate_valid = 0U;
        *reliable_distance = raw_distance;
        return 1U;
    }

    *reliable_distance = s_distance_last_valid_distance;
    return 1U;
}

static float DistanceServo_SlewLimit(float current_ref, float target_ref, float dt)
{
    float slope = DIST_SERVO_ACCEL_RPM_PER_S;
    float max_delta = 0.0f;
    float delta = 0.0f;

    if ((current_ref * target_ref) < 0.0f || fabsf(target_ref) < fabsf(current_ref)) {
        slope = DIST_SERVO_DECEL_RPM_PER_S;
    }

    max_delta = slope * dt;
    delta = Limit_Float(target_ref - current_ref, -max_delta, max_delta);
    return current_ref + delta;
}

void DistanceServo_Reset(DJI_t *motor)
{
    if (motor == NULL || distance_planner.motor == motor) {
        memset(&distance_planner, 0, sizeof(distance_planner));
        memset(&g_distance_servo_debug, 0, sizeof(g_distance_servo_debug));
        s_distance_last_valid_distance = 0.0f;
        s_distance_last_valid_tick = 0U;
        s_distance_jump_candidate_distance = 0.0f;
        s_distance_last_valid_ready = 0U;
        s_distance_jump_candidate_valid = 0U;
        s_distance_last_raw_distance = 0.0f;
        s_distance_last_raw_tick = 0U;
        s_distance_jump_candidate_tick = 0U;
        s_distance_last_raw_ready = 0U;
    }
}

uint8_t DistanceServo_IsArrived(DJI_t *motor)
{
    if (!distance_planner.initialized) return 0U;
    if (distance_planner.motor != motor) return 0U;
    return distance_planner.arrived_confirmed;
}

float Distance_Speed_Plan(float target_distance, float current_distance, DJI_t *motor)
{
    uint32_t now_tick = 0U;
    float dt = 0.0f;
    float desired_speed_ref = 0.0f;
    float reliable_distance_debug = current_distance;
    float filtered_distance_debug = current_distance;
    float control_distance_debug = current_distance;

    if (motor == NULL) return 0.0f;

    now_tick = HAL_GetTick();
    dt = DistanceServo_GetDt(now_tick);

    if (!distance_planner.initialized || distance_planner.motor != motor) {
        memset(&distance_planner, 0, sizeof(distance_planner));
        distance_planner.motor = motor;
        distance_planner.initialized = 1U;
        distance_planner.target_distance = target_distance;
        distance_planner.last_speed_ref = motor->speedPID.ref;
        distance_planner.last_error = target_distance - current_distance;
        distance_planner.last_tick = now_tick;
        distance_planner.filtered_distance = current_distance;

        s_distance_last_valid_distance = current_distance;
        s_distance_last_valid_tick = now_tick;
        s_distance_last_valid_ready =
            (uint8_t)(Distance_Is_Valid(current_distance) &&
                      current_distance >= DIST_SERVO_EFFECTIVE_MIN_MM &&
                      current_distance <= DIST_SERVO_EFFECTIVE_MAX_MM);
        s_distance_jump_candidate_distance = current_distance;
        s_distance_jump_candidate_valid = 0U;
        s_distance_last_raw_distance = current_distance;
        s_distance_last_raw_tick = now_tick;
        s_distance_jump_candidate_tick = now_tick;
        s_distance_last_raw_ready = 1U;
    }

    if (fabsf(target_distance - distance_planner.target_distance) > DIST_SERVO_TARGET_CHANGE_TOL_MM) {
        distance_planner.target_distance = target_distance;
        distance_planner.arrived = 0U;
        distance_planner.arrived_confirmed = 0U;
        distance_planner.use_reliable_near_target = 0U;
    }

    if (Float_Is_Usable(target_distance)) {
        float reliable_distance = 0.0f;
        if (!DistanceServo_GetReliableSample(current_distance, now_tick, &reliable_distance)) {
            desired_speed_ref = 0.0f;
            distance_planner.arrived = 0U;
            distance_planner.arrived_confirmed = 0U;
            distance_planner.arrived_tick = 0U;
        } else {
        float filtered_distance = LowPass_Filter(
            reliable_distance,
            &distance_planner.filtered_distance,
            DIST_FILTER_ALPHA
        );
        float reliable_error = target_distance - reliable_distance;
        float control_distance = filtered_distance;
        float error = 0.0f;
        float abs_error = fabsf(error);
        float effective_tol = DIST_SERVO_POS_TOL_MM;
        float stop_error = 0.0f;

        if (distance_planner.use_reliable_near_target) {
            if (fabsf(reliable_error) > DIST_SERVO_NEAR_SWITCH_OUT_MM) {
                distance_planner.use_reliable_near_target = 0U;
            }
        } else {
            if (fabsf(reliable_error) < DIST_SERVO_NEAR_SWITCH_IN_MM) {
                distance_planner.use_reliable_near_target = 1U;
            }
        }

        if (distance_planner.use_reliable_near_target) {
            control_distance = reliable_distance;
        }

        error = target_distance - control_distance;
        abs_error = fabsf(error);
        reliable_distance_debug = reliable_distance;
        filtered_distance_debug = filtered_distance;
        control_distance_debug = control_distance;

        if (distance_planner.arrived) {
            effective_tol = DIST_SERVO_POS_TOL_MM * 1.5f;
        }

        stop_error = abs_error - effective_tol;
        if (stop_error < 0.0f) stop_error = 0.0f;

        if (abs_error <= effective_tol) {
            desired_speed_ref = error * DIST_SERVO_HOLD_KP_RPM_PER_MM;
            desired_speed_ref = Limit_Float(
                desired_speed_ref,
                -DIST_SERVO_HOLD_MAX_SPEED_RPM,
                DIST_SERVO_HOLD_MAX_SPEED_RPM
            );

            if (abs_error < 0.5f) {
                desired_speed_ref = 0.0f;
            }

            if (!distance_planner.arrived) {
                if (distance_planner.arrived_tick == 0U) {
                    distance_planner.arrived_tick = now_tick;
                } else if ((now_tick - distance_planner.arrived_tick) >= DIST_SERVO_STABLE_MS) {
                    distance_planner.arrived = 1U;
                    distance_planner.arrived_confirmed = 1U;
                }
            }
        } else {
            float max_speed = DIST_SERVO_MAX_SPEED_RPM;
            float speed_by_p = 0.0f;
            float speed_by_brake = 0.0f;
            float speed_mag = 0.0f;

            distance_planner.arrived = 0U;
            distance_planner.arrived_confirmed = 0U;
            distance_planner.arrived_tick = 0U;

            if (motor->posPID.outputMax > 1.0f && motor->posPID.outputMax < max_speed) {
                max_speed = motor->posPID.outputMax;
            }

            speed_by_p = DIST_SERVO_KP_RPM_PER_MM * stop_error;
            speed_by_brake = sqrtf(2.0f * DIST_SERVO_BRAKE_GAIN_RPM2_PER_MM * stop_error);
            speed_mag = speed_by_p;

            if (speed_by_brake < speed_mag) speed_mag = speed_by_brake;
            speed_mag = Limit_Float(speed_mag, 0.0f, max_speed);

            if (stop_error < 50.0f && speed_mag > DIST_SERVO_APPROACH_SPEED_RPM) {
                speed_mag = DIST_SERVO_APPROACH_SPEED_RPM;
            }

            if (speed_mag < DIST_SERVO_MIN_MOVE_RPM) {
                speed_mag = DIST_SERVO_MIN_MOVE_RPM;
            }

            desired_speed_ref = Sign_Float(error) * speed_mag;
        }

        distance_planner.last_error = error;
        }
    } else {
        desired_speed_ref = 0.0f;
        distance_planner.arrived = 0U;
        distance_planner.arrived_confirmed = 0U;
        distance_planner.arrived_tick = 0U;
    }

    desired_speed_ref = Abs_Limit_Float(desired_speed_ref, DIST_SERVO_MAX_SPEED_RPM);
    distance_planner.last_speed_ref =
        DistanceServo_SlewLimit(distance_planner.last_speed_ref, desired_speed_ref, dt);

    if (fabsf(distance_planner.last_speed_ref) < 1.0f && desired_speed_ref == 0.0f) {
        distance_planner.last_speed_ref = 0.0f;
    }

    g_distance_servo_debug.target_distance = target_distance;
    g_distance_servo_debug.raw_distance = current_distance;
    g_distance_servo_debug.reliable_distance = reliable_distance_debug;
    g_distance_servo_debug.filtered_distance = filtered_distance_debug;
    g_distance_servo_debug.control_distance = control_distance_debug;
    g_distance_servo_debug.desired_speed_ref = distance_planner.last_speed_ref;
    g_distance_servo_debug.motor_rpm = motor->FdbData.rpm;
    g_distance_servo_debug.use_reliable_near_target = distance_planner.use_reliable_near_target;
    g_distance_servo_debug.arrived = distance_planner.arrived;
    g_distance_servo_debug.arrived_confirmed = distance_planner.arrived_confirmed;

    distance_planner.last_tick = now_tick;
    return distance_planner.last_speed_ref;
}

void Distance_servo(float target_distance, DJI_t *motor)
{
    float current_distance = 0.0f;
    float speed_ref = 0.0f;

    if (motor == NULL) return;

    current_distance = lidar.distance_aver;
    speed_ref = Distance_Speed_Plan(target_distance, current_distance, motor);

    motor->posPID.ref = target_distance;
    motor->posPID.fdb = current_distance;
    motor->posPID.cur_error = target_distance - current_distance;
    motor->posPID.error[0] = motor->posPID.error[1];
    motor->posPID.error[1] = motor->posPID.cur_error;
    motor->posPID.output = speed_ref;

    motor->speedPID.ref = speed_ref;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
}

void PID_Clear(PID_t *pid)
{
    if (pid == NULL) return;

    pid->ref = 0.0f;
    pid->fdb = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->cur_error = 0.0f;
    pid->output = 0.0f;
}

void Motor_State_Reset(DJI_t *motor)
{
    if (motor == NULL) return;

    PID_Clear(&motor->posPID);
    PID_Clear(&motor->speedPID);
    DistanceServo_Reset(motor);
    if (motor == &hDJI[2]) YawServo_Reset();
    if (motor == &hDJI[3]) ArmServo_Reset();
}

void pr(void)
{
    printf("offset_distance:%d,distance:%d\r\n", (int)distance_offset, (int)lidar.distance);
}

void Reset_DJI_Motor_Full(DJI_t *ptr)
{
    if (ptr == NULL) return;

    ptr->AxisData.AxisAngle_inDegree = 0.0f;
    ptr->Calculate.RotorAngle_all = 0.0f;
    ptr->Calculate.RotorRound = 0;
    ptr->Calculate.RotorAngle_0_360_OffSet = ptr->FdbData.RotorAngle_0_360;
    ptr->Calculate.RotorAngle_0_360_Log[LAST] = ptr->FdbData.RotorAngle_0_360;
    ptr->Calculate.RotorAngle_0_360_Log[NOW] = ptr->FdbData.RotorAngle_0_360;

    PID_Clear(&ptr->posPID);
    PID_Clear(&ptr->speedPID);
    DistanceServo_Reset(ptr);
    if (ptr == &hDJI[2]) YawServo_Reset();
    if (ptr == &hDJI[3]) ArmServo_Reset();
}

typedef struct {
    DJI_t *motor;
    uint8_t initialized;
    uint8_t arrived;
    uint8_t arrived_confirmed;
    float target_degree;
    uint8_t use_position_servo;
    uint8_t force_lock_active;
    float force_lock_degree;
    float last_speed_ref;
    float filtered_degree;
    uint32_t last_tick;
    uint32_t arrived_tick;
} YawPlanner_t;

static YawPlanner_t yaw_planner = {0};

static float YawServo_SlewLimit(DJI_t *motor, float current_ref, float target_ref, float dt)
{
    float slope_deg_per_s2 = YAW_ACCEL_DEG_PER_S2;
    float slope_rpm_per_s = 0.0f;
    float max_delta = 0.0f;
    float delta = 0.0f;

    if (motor == NULL) return target_ref;

    if ((current_ref * target_ref) < 0.0f || fabsf(target_ref) < fabsf(current_ref)) {
        slope_deg_per_s2 = YAW_BRAKE_GAIN_DEG_PER_S2;
    }

    slope_rpm_per_s = slope_deg_per_s2 * motor->reductionRate / 6.0f;
    max_delta = slope_rpm_per_s * dt;
    delta = Limit_Float(target_ref - current_ref, -max_delta, max_delta);
    return current_ref + delta;
}

typedef struct {
    uint8_t initialized;
    uint8_t arrived;
    uint8_t arrived_confirmed;
    float target_degree;
    float initial_angle;
    float start_time;
    float max_speed;
    float accel_time;
    float const_time;
    float total_time;
    float direction;
    float last_planned_angle;
} ArmPlanner_t;

static ArmPlanner_t arm_planner = {0};

void YawServo_Reset(void)
{
    memset(&yaw_planner, 0, sizeof(yaw_planner));
}

uint8_t YawServo_IsArrived(void)
{
    return yaw_planner.arrived_confirmed;
}

void YawServo_ForceLockCurrent(DJI_t *motor)
{
    if (motor == NULL) return;

    yaw_planner.force_lock_active = 1U;
    yaw_planner.force_lock_degree = motor->AxisData.AxisAngle_inDegree;
    yaw_planner.motor = motor;
    PID_Clear(&motor->posPID);
    PID_Clear(&motor->speedPID);
}

void ArmServo_Reset(void)
{
    memset(&arm_planner, 0, sizeof(arm_planner));
}

uint8_t ArmServo_IsArrived(void)
{
    return arm_planner.arrived_confirmed;
}

static void Yaw_Plan_Update(float target_degree, float current_degree, uint32_t now_tick)
{
    (void)now_tick;

    if (!yaw_planner.initialized || yaw_planner.motor == NULL) {
        yaw_planner.initialized = 1U;
        yaw_planner.motor = NULL;
        yaw_planner.target_degree = target_degree;
        yaw_planner.use_position_servo = (uint8_t)(fabsf(target_degree - current_degree) < 180.0f);
        yaw_planner.last_speed_ref = 0.0f;
        yaw_planner.filtered_degree = current_degree;
        yaw_planner.last_tick = HAL_GetTick();
        yaw_planner.arrived_tick = 0U;
        yaw_planner.arrived = 0U;
        yaw_planner.arrived_confirmed = 0U;
    }

    if (fabsf(target_degree - yaw_planner.target_degree) > 0.5f) {
        yaw_planner.target_degree = target_degree;
        yaw_planner.use_position_servo = (uint8_t)(fabsf(target_degree - current_degree) < 180.0f);
        yaw_planner.arrived = 0U;
        yaw_planner.arrived_confirmed = 0U;
        yaw_planner.arrived_tick = 0U;
    }

    yaw_planner.filtered_degree = current_degree;
}

void Yaw_servo(float target_degree, DJI_t *motor)
{
    uint32_t now_tick = 0U;
    float current_degree = 0.0f;
    float target_error = 0.0f;
    float abs_target_error = 0.0f;
    float desired_axis_speed = 0.0f;
    float desired_motor_rpm = 0.0f;
    float dt = 0.0f;
    float max_speed_deg_per_s = YAW_MAX_SPEED_DEG_PER_S;
    float speed_by_p = 0.0f;
    float speed_by_brake = 0.0f;
    float speed_mag = 0.0f;
    float stop_error = 0.0f;
    uint8_t sign_positive = 0U;

    if (motor == NULL) return;

    now_tick = HAL_GetTick();
    current_degree = motor->AxisData.AxisAngle_inDegree;
    Yaw_Plan_Update(target_degree, current_degree, now_tick);

    if (yaw_planner.force_lock_active && yaw_planner.motor == motor) {
        positionServo(yaw_planner.force_lock_degree, motor);
        yaw_planner.last_tick = now_tick;
        yaw_planner.arrived = 1U;
        yaw_planner.arrived_confirmed = 1U;
        yaw_planner.arrived_tick = now_tick;
        return;
    }

    if (yaw_planner.use_position_servo) {
        positionServo(target_degree, motor);
        yaw_planner.last_tick = now_tick;
        yaw_planner.motor = motor;
        yaw_planner.arrived = (uint8_t)(fabsf(target_degree - current_degree) <= YAW_POS_TOL_DEG);
        yaw_planner.arrived_confirmed = yaw_planner.arrived;
        if (yaw_planner.arrived) {
            yaw_planner.arrived_tick = now_tick;
        } else {
            yaw_planner.arrived_tick = 0U;
        }
        return;
    }

    if (yaw_planner.last_tick == 0U) {
        dt = 0.001f;
    } else {
        uint32_t dt_ms = now_tick - yaw_planner.last_tick;
        dt = (float)dt_ms * 0.001f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.020f) dt = 0.020f;
    }

    target_error = target_degree - current_degree;
    abs_target_error = fabsf(target_error);

    if (target_error > 0.0f) {
        sign_positive = 1U;
    }

    if (abs_target_error <= YAW_POS_TOL_DEG) {
        desired_axis_speed = target_error * YAW_HOLD_KP_DEG_PER_S_PER_DEG;
        desired_axis_speed = Limit_Float(
            desired_axis_speed,
            -YAW_HOLD_MAX_SPEED_DEG_PER_S,
            YAW_HOLD_MAX_SPEED_DEG_PER_S
        );

        if (abs_target_error < YAW_HOLD_ZERO_TOL_DEG) {
            desired_axis_speed = 0.0f;
        }

        if (yaw_planner.arrived_tick == 0U) {
            yaw_planner.arrived_tick = now_tick;
        } else if ((now_tick - yaw_planner.arrived_tick) >= YAW_STABLE_MS) {
            yaw_planner.arrived = 1U;
            yaw_planner.arrived_confirmed = 1U;
        }
    } else {
        stop_error = abs_target_error - YAW_POS_TOL_DEG;
        if (stop_error < 0.0f) stop_error = 0.0f;

        if (stop_error < 18.0f) {
            max_speed_deg_per_s = YAW_APPROACH_SPEED_DEG_PER_S;
        }

        speed_by_p = YAW_MOVE_KP_DEG_PER_S_PER_DEG * stop_error;
        speed_by_brake = sqrtf(2.0f * YAW_BRAKE_GAIN_DEG_PER_S2 * stop_error);
        speed_mag = speed_by_p;
        if (speed_by_brake < speed_mag) speed_mag = speed_by_brake;
        speed_mag = Limit_Float(speed_mag, 0.0f, max_speed_deg_per_s);

        if (stop_error > 6.0f && speed_mag < YAW_MIN_MOVE_DEG_PER_S) {
            speed_mag = YAW_MIN_MOVE_DEG_PER_S;
        }

        desired_axis_speed = (sign_positive != 0U) ? speed_mag : -speed_mag;
        yaw_planner.arrived = 0U;
        yaw_planner.arrived_confirmed = 0U;
        yaw_planner.arrived_tick = 0U;
    }

    desired_motor_rpm = desired_axis_speed * motor->reductionRate / 6.0f;
    desired_motor_rpm = Abs_Limit_Float(desired_motor_rpm, YAW_MAX_SPEED_DEG_PER_S * motor->reductionRate / 6.0f);
    yaw_planner.last_speed_ref = YawServo_SlewLimit(motor, yaw_planner.last_speed_ref, desired_motor_rpm, dt);

    if (fabsf(yaw_planner.last_speed_ref) < 1.0f && desired_motor_rpm == 0.0f) {
        yaw_planner.last_speed_ref = 0.0f;
    }

    yaw_planner.last_tick = now_tick;
    yaw_planner.motor = motor;

    motor->posPID.ref = target_degree;
    motor->posPID.fdb = current_degree;
    motor->posPID.cur_error = target_error;
    motor->posPID.output = desired_axis_speed;

    motor->speedPID.ref = yaw_planner.last_speed_ref;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
}

static void Arm_Plan_Update(float target_degree, float current_degree, uint32_t now_tick)
{
    if (!arm_planner.initialized || fabsf(target_degree - arm_planner.target_degree) > 0.5f) {
        float angle_diff = target_degree - current_degree;
        float abs_diff = fabsf(angle_diff);
        float accel_time = ARM_MAX_SPEED_DEG_PER_S / ARM_ACCEL_DEG_PER_S2;
        float accel_dist = 0.5f * ARM_ACCEL_DEG_PER_S2 * accel_time * accel_time;
        float const_time = (abs_diff - 2.0f * accel_dist) / ARM_MAX_SPEED_DEG_PER_S;

        arm_planner.target_degree = target_degree;
        arm_planner.initial_angle = current_degree;
        arm_planner.start_time = (float)now_tick;
        arm_planner.arrived = 0U;
        arm_planner.arrived_confirmed = 0U;
        arm_planner.last_planned_angle = current_degree;
        arm_planner.direction = (angle_diff > 0.0f) ? 1.0f : -1.0f;

        if (const_time > 0.0f) {
            arm_planner.max_speed = ARM_MAX_SPEED_DEG_PER_S;
            arm_planner.accel_time = accel_time;
            arm_planner.const_time = const_time;
            arm_planner.total_time = 2.0f * accel_time + const_time;
        } else {
            float v_peak = sqrtf(abs_diff * ARM_ACCEL_DEG_PER_S2);
            arm_planner.max_speed = v_peak;
            arm_planner.accel_time = v_peak / ARM_ACCEL_DEG_PER_S2;
            arm_planner.const_time = 0.0f;
            arm_planner.total_time = 2.0f * arm_planner.accel_time;
        }

        arm_planner.initialized = 1U;
    }

    {
        float elapsed = ((float)now_tick - arm_planner.start_time) * 0.001f;
        float planned_angle = 0.0f;

        if (elapsed >= arm_planner.total_time) {
            planned_angle = arm_planner.target_degree;
        } else if (elapsed <= arm_planner.accel_time) {
            planned_angle = arm_planner.initial_angle +
                            arm_planner.direction * 0.5f * ARM_ACCEL_DEG_PER_S2 * elapsed * elapsed;
        } else if (elapsed <= arm_planner.accel_time + arm_planner.const_time) {
            float t_acc = arm_planner.accel_time;
            float accel_dist = 0.5f * ARM_ACCEL_DEG_PER_S2 * t_acc * t_acc;
            planned_angle = arm_planner.initial_angle +
                            arm_planner.direction * (accel_dist + arm_planner.max_speed * (elapsed - t_acc));
        } else {
            float t_acc = arm_planner.accel_time;
            float t_const = arm_planner.const_time;
            float accel_dist = 0.5f * ARM_ACCEL_DEG_PER_S2 * t_acc * t_acc;
            float const_dist = arm_planner.max_speed * t_const;
            float t_dec = elapsed - t_acc - t_const;
            planned_angle = arm_planner.initial_angle +
                            arm_planner.direction *
                                (accel_dist + const_dist +
                                 arm_planner.max_speed * t_dec -
                                 0.5f * ARM_ACCEL_DEG_PER_S2 * t_dec * t_dec);
        }

        arm_planner.last_planned_angle = planned_angle;
    }

    if (fabsf(target_degree - current_degree) <= ARM_POS_TOL_DEG) {
        arm_planner.arrived = 1U;
        arm_planner.arrived_confirmed = 1U;
    } else {
        arm_planner.arrived = 0U;
        arm_planner.arrived_confirmed = 0U;
    }
}

void Arm_servo(float target_degree, DJI_t *motor)
{
    uint32_t now_tick = 0U;
    float current_degree = 0.0f;

    if (motor == NULL) return;

    now_tick = HAL_GetTick();
    current_degree = motor->AxisData.AxisAngle_inDegree;

    Arm_Plan_Update(target_degree, current_degree, now_tick);
    positionServo(arm_planner.last_planned_angle, motor);
}
