/*pid算法和速度位置伺服*/

#include "Caculate.h"
#include "math.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"

//增量式PID算法
void PID_Calc(PID_t *pid){
    if (pid == NULL) return;

    pid->cur_error = pid->ref - pid->fdb;
    pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->ref - pid->fdb;
    /*设定输出上限*/
    if(pid->output > pid->outputMax) pid->output = pid->outputMax;
    if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
    /*输出死区：输出值太小则直接置零，防止微颤*/
    if(pid->outputMax > 0.0f && pid->outputMin > 0.0f && fabsf(pid->output) < pid->outputMin)
        pid->output = 0.0f;

}

//比例算法
void P_Calc(PID_t *pid){
    if (pid == NULL) return;

    pid->cur_error = pid->ref - pid->fdb;
    pid->output = pid->KP * pid->cur_error;
    /*设定输出上限*/
    if(pid->output > pid->outputMax) pid->output = pid->outputMax;
    if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;

    if(pid->outputMin > 0.0f && fabsf(pid->output)<pid->outputMin)
        pid->output=0;

}

//位置伺服函数
void positionServo(float ref, DJI_t * motor){
    if (motor == NULL) return;

    motor->posPID.ref = ref;
    motor->posPID.fdb = motor->AxisData.AxisAngle_inDegree;
    PID_Calc(&motor->posPID);

    motor->speedPID.ref = motor->posPID.output;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
    // 限制 speed output 绝对值 >= 800，避免输出过小导致电机蠕动
}

//速度伺服函数
void speedServo(float ref, DJI_t * motor){
    if (motor == NULL) return;

    motor->speedPID.ref = ref;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
}

typedef struct {
    DJI_t *motor;
    uint8_t initialized;
    uint8_t arrived;
    uint8_t arrived_confirmed;      // 稳定计时确认后的到达标志
    float target_distance;
    float last_speed_ref;
    float last_error;
    uint32_t last_tick;
    /* 启动助力（Kick-Start）状态 */
    uint8_t kick_start_active;
    uint32_t kick_start_start_tick;

    /* 一阶低通滤波 */
    float filtered_distance;         // 滤波后的距离值

    /* 到达稳定计时 */
    uint32_t arrived_tick;           // 进入死区的时刻
} DistanceServoPlanner_t;

static DistanceServoPlanner_t distance_planner = {0};

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
    if (!Float_Is_Usable(distance)) return 0;
    if (distance < DIST_SERVO_SENSOR_MIN_MM) return 0;
    if (distance > DIST_SERVO_SENSOR_MAX_MM) return 0;
    return 1;
}

static float DistanceServo_GetDt(uint32_t now_tick)
{
    if (!distance_planner.initialized || distance_planner.last_tick == 0U) {
        return DIST_SERVO_DEFAULT_DT_S;
    }

    uint32_t dt_ms = now_tick - distance_planner.last_tick;
    if (dt_ms == 0U) {
        return DIST_SERVO_DEFAULT_DT_S;
    }

    float dt = (float)dt_ms * 0.001f;
    if (dt > DIST_SERVO_MAX_DT_S) dt = DIST_SERVO_MAX_DT_S;
    if (dt < DIST_SERVO_DEFAULT_DT_S) dt = DIST_SERVO_DEFAULT_DT_S;
    return dt;
}

/**
 * @brief 一阶低通滤波，平滑雷达噪声
 * @param raw      原始测量值
 * @param filtered 上一轮滤波值（会被写回）
 * @param alpha    滤波系数 (0~1)，越小越平滑
 * @return 滤波后的值
 */
static float LowPass_Filter(float raw, float *filtered, float alpha)
{
    *filtered = alpha * raw + (1.0f - alpha) * (*filtered);
    return *filtered;
}

static float DistanceServo_SlewLimit(float current_ref, float target_ref, float dt)
{
    float slope = DIST_SERVO_ACCEL_RPM_PER_S;

    /* 同号减速或换向时使用较大的减速度，防止临近目标还拖着速度前冲 */
    if ((current_ref * target_ref) < 0.0f || fabsf(target_ref) < fabsf(current_ref)) {
        slope = DIST_SERVO_DECEL_RPM_PER_S;
    }

    float max_delta = slope * dt;
    float delta = Limit_Float(target_ref - current_ref, -max_delta, max_delta);
    return current_ref + delta;
}

void DistanceServo_Reset(DJI_t *motor)
{
    if (motor == NULL || distance_planner.motor == motor) {
        memset(&distance_planner, 0, sizeof(distance_planner));
    }
}

uint8_t DistanceServo_IsArrived(DJI_t *motor)
{
    if (!distance_planner.initialized) return 0;
    if (distance_planner.motor != motor) return 0;
    return distance_planner.arrived_confirmed;
}

/**
 * @brief  基于 distance 误差生成平滑 speedPID.ref。
 * @note   目标突变时不会直接跳到新速度；速度给定会按加速度/减速度限制连续变化。
 *         靠近目标时采用刹车曲线和爬行速度，避免高速冲过目标，也避免因速度给定过小提前停住。
 * @param  target_distance   目标距离，单位 mm
 * @param  current_distance  当前测距，单位 mm
 * @param  motor             被控电机指针；speedPID.ref 单位仍为电机转子 rpm
 * @retval 规划后的速度给定，单位 rpm
 */
float Distance_Speed_Plan(float target_distance, float current_distance, DJI_t *motor)
{
    if (motor == NULL) return 0.0f;

    uint32_t now_tick = HAL_GetTick();
    float dt = DistanceServo_GetDt(now_tick);

        if (!distance_planner.initialized || distance_planner.motor != motor) {
        memset(&distance_planner, 0, sizeof(distance_planner));
        distance_planner.motor = motor;
        distance_planner.initialized = 1U;
        distance_planner.target_distance = target_distance;
        distance_planner.last_speed_ref = motor->speedPID.ref;
        distance_planner.last_error = target_distance - current_distance;
        distance_planner.last_tick = now_tick;
        distance_planner.filtered_distance = current_distance; // 初始化滤波值
    }

    if (fabsf(target_distance - distance_planner.target_distance) > DIST_SERVO_TARGET_CHANGE_TOL_MM) {
        /* 目标改变时只更新目标，不清零 last_speed_ref，保证速度给定连续过渡 */
        distance_planner.target_distance = target_distance;
        distance_planner.arrived = 0U;
        distance_planner.arrived_confirmed = 0U;
    }

        float desired_speed_ref = 0.0f;

    if (Distance_Is_Valid(current_distance) && Float_Is_Usable(target_distance)) {
        /* 一阶低通滤波，平滑雷达测量值和机构晃动的影响 */
        float filtered_distance = LowPass_Filter(
            current_distance,
            &distance_planner.filtered_distance,
            DIST_FILTER_ALPHA
        );

        float error = target_distance - filtered_distance;
        float abs_error = fabsf(error);

        /* 迟滞比较器：进入死区后需更大误差才能退出，避免边界震荡 */
        float effective_tol = DIST_SERVO_POS_TOL_MM;
        if (distance_planner.arrived) {
            effective_tol = DIST_SERVO_POS_TOL_MM * 1.5f;  // 退出死区阈值放大
        }

        float stop_error = abs_error - effective_tol;
        if (stop_error < 0.0f) stop_error = 0.0f;

        if (abs_error <= effective_tol) {
            desired_speed_ref = 0.0f;

            /* 稳定计时确认到达：持续在死区内 DIST_SERVO_STABLE_MS 毫秒才确认 */
            if (!distance_planner.arrived) {
                if (distance_planner.arrived_tick == 0U) {
                    distance_planner.arrived_tick = now_tick;
                } else if ((now_tick - distance_planner.arrived_tick) >= DIST_SERVO_STABLE_MS) {
                    distance_planner.arrived = 1U;
                    distance_planner.arrived_confirmed = 1U;
                }
            }
        } else {
            /* 退出死区：清除到达标记和计时 */
            distance_planner.arrived = 0U;
            distance_planner.arrived_confirmed = 0U;
            distance_planner.arrived_tick = 0U;
            float max_speed = DIST_SERVO_MAX_SPEED_RPM;
            if (motor->posPID.outputMax > 1.0f && motor->posPID.outputMax < max_speed) {
                max_speed = motor->posPID.outputMax;
            }

            float speed_by_p = DIST_SERVO_KP_RPM_PER_MM * stop_error;
            float speed_by_brake = sqrtf(2.0f * DIST_SERVO_BRAKE_GAIN_RPM2_PER_MM * stop_error);
            float speed_mag = speed_by_p;

            if (speed_by_brake < speed_mag) speed_mag = speed_by_brake;
            speed_mag = Limit_Float(speed_mag, 0.0f, max_speed);

            if (speed_mag < DIST_SERVO_MIN_MOVE_RPM) {
                speed_mag = DIST_SERVO_MIN_MOVE_RPM;
            }

            desired_speed_ref = Sign_Float(error) * speed_mag;
            distance_planner.arrived = 0U;
        }

        distance_planner.last_error = error;
    } else {
        /* 测距无效时不继续追目标，速度给定按减速度回零，避免传感器异常导致机构乱跑 */
        desired_speed_ref = 0.0f;
        distance_planner.arrived = 0U;
    }

    desired_speed_ref = Abs_Limit_Float(desired_speed_ref, DIST_SERVO_MAX_SPEED_RPM);
    distance_planner.last_speed_ref = DistanceServo_SlewLimit(distance_planner.last_speed_ref, desired_speed_ref, dt);

    /* === 启动助力 Kick-Start：当有速度指令但电机实际没转时，短时叠加冲击 === */
    if (fabsf(desired_speed_ref) > DIST_SERVO_MIN_MOVE_RPM) {
        /* 有运动意图 */
        if (fabsf(motor->FdbData.rpm) < KICK_START_THRESHOLD_RPM) {
            /* 电机没转起来 → 启动 kick-start */
            if (!distance_planner.kick_start_active) {
                distance_planner.kick_start_active = 1U;
                distance_planner.kick_start_start_tick = now_tick;
            }
        } else {
            /* 电机已经转起来了 → 关闭 kick-start */
            distance_planner.kick_start_active = 0U;
        }
    } else {
        /* 没有运动意图 → 关闭 kick-start */
        distance_planner.kick_start_active = 0U;
    }

    if (distance_planner.kick_start_active) {
        uint32_t kick_elapsed = now_tick - distance_planner.kick_start_start_tick;
        if (kick_elapsed < KICK_START_MAX_DURATION_MS) {
            /* 叠加一个冲击速度（方向与 desired_speed_ref 相同） */
            float boost = Sign_Float(desired_speed_ref) * KICK_START_BOOST_RPM;
            distance_planner.last_speed_ref += boost;
            /* 限幅到最大速度 */
            distance_planner.last_speed_ref = Limit_Float(
                distance_planner.last_speed_ref,
                -DIST_SERVO_MAX_SPEED_RPM,
                DIST_SERVO_MAX_SPEED_RPM
            );
        } else {
            /* 超时关闭 kick-start */
            distance_planner.kick_start_active = 0U;
        }
    }
    /* === Kick-Start 结束 === */

    if (fabsf(distance_planner.last_speed_ref) < 1.0f && desired_speed_ref == 0.0f) {
        distance_planner.last_speed_ref = 0.0f;
    }

    distance_planner.last_tick = now_tick;
    return distance_planner.last_speed_ref;
}

/**
 * @param  target_distance  目标距离（mm），正值远离、负值靠近
 * @param  motor 被控电机指针
 */
void Distance_servo(float target_distance, DJI_t * motor) {
    if (motor == NULL) return;

    float current_distance = lidar.distance_aver;
    float speed_ref = Distance_Speed_Plan(target_distance, current_distance, motor);

    /* 保留 posPID 中的调试信息，但不再用增量式位置 PID 直接生成速度，避免目标突变时速度给定跃变 */
    motor->posPID.ref = target_distance;
    motor->posPID.fdb = current_distance;
    motor->posPID.cur_error = target_distance - current_distance;
    motor->posPID.error[0] = motor->posPID.error[1];
    motor->posPID.error[1] = motor->posPID.cur_error;
    motor->posPID.output = speed_ref;

    motor->speedPID.ref = speed_ref;
    motor->speedPID.fdb = motor->FdbData.rpm;

        if (DistanceServo_IsArrived(motor)) {
        /* 到达后：同时清理位置环和速度环的增量累加，防止积分/累加项残留导致抖动 */
        PID_Clear(&motor->speedPID);
        PID_Clear(&motor->posPID);
        motor->speedPID.ref = 0.0f;
        motor->speedPID.fdb = motor->FdbData.rpm;
        motor->speedPID.output = 0.0f;
        motor->posPID.output = 0.0f;
        return;
    }

    PID_Calc(&motor->speedPID);
}

/**
 * @brief  清除 PID 的动态计算变量（保留 KP, KI, KD 和限幅等静态参数）
 * @param  pid: 指向 PID 结构体的指针
 */
void PID_Clear(PID_t *pid) {
    if (pid == NULL) return;

    pid->ref = 0.0f;       // 清除目标值
    pid->fdb = 0.0f;       // 清除反馈值
    // 清除历史误差 (如果是增量式PID，通常有 error[0], error[1], 可能还有 error[2])
    pid->error[0] = 0.0f;  // 当前误差
    pid->error[1] = 0.0f;  // 上次误差
    // pid->error[2] = 0.0f;  // 上上次误差 (如果你的结构体里有这个变量，取消注释)

    // 清除当前误差与输出（增量式 PID 的 output 是累加量）
    pid->cur_error = 0.0f;
    pid->output = 0.0f;
    // 如果你使用了位置式 PID，请务必清除积分项！
    // pid->integral = 0.0f;

}

/**
 * @brief  复位大疆电机的全部动态控制状态（适用于跳出循环后的复位）
 * @param  motor: 指向大疆电机结构体的指针
 */
void Motor_State_Reset(DJI_t *motor) {
    if (motor == NULL) return;

    // 1. 复位位置环（外环）
    PID_Clear(&motor->posPID);

    // 2. 复位速度环（内环）
    PID_Clear(&motor->speedPID);

    // 3. 如果复位的是 distance 伺服电机，同步清除速度规划器状态
    DistanceServo_Reset(motor);

    // 4. (可选) 如果你需要在代码里记录当前电机的多圈绝对角度，可能需要重新获取一下 offset
    // motor->encode_offset = motor->encode; // 视你的具体底层逻辑而定
}

void pr(void){

        printf("offset_distance:%d,distance:%d\r\n",(int)distance_offset,(int)lidar.distance);

        /* code */
}
/**
 * @brief  重置指定大疆电机的累计位置和PID状态
 * @param  ptr: 电机结构体指针 (例如 &hDJI[3])
 */
/**
 * @brief  彻底重置大疆电机的累计角度、圈数及PID状态
 * @param  ptr: 电机结构体指针 (如 &hDJI[3])
 */
void Reset_DJI_Motor_Full(DJI_t *ptr) {
    if (ptr == NULL) return;

    // --- 1. 位置计算逻辑清零 ---
    // 重置电机轴输出角度和多圈数据[cite: 1, 4]
    ptr->AxisData.AxisAngle_inDegree = 0.0f;
    ptr->Calculate.RotorAngle_all = 0.0f;
    ptr->Calculate.RotorRound = 0; // 圈数重置[cite: 1, 4]

    // 将当前的机械角度设置为新的偏移起点
    // 这样下次 DJI_Update 计算时，当前位置将被视为 0 度
    ptr->Calculate.RotorAngle_0_360_OffSet = ptr->FdbData.RotorAngle_0_360;
    ptr->Calculate.RotorAngle_0_360_Log[LAST] = ptr->FdbData.RotorAngle_0_360;
    ptr->Calculate.RotorAngle_0_360_Log[NOW] = ptr->FdbData.RotorAngle_0_360;

    // --- 2. PID 控制器状态清零[cite: 1] ---
    // 使用统一函数清除，避免重复实现
    PID_Clear(&ptr->posPID);
    PID_Clear(&ptr->speedPID);
    DistanceServo_Reset(ptr);
}



