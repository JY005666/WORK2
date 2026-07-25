#ifndef _CACULATE_H__
#define _CACULATE_H__

#include "DJI.h"
#include "Stp23L.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stdio.h"

#define DIST_FILTER_ALPHA             0.03f
#define DIST_OFFSET_SAMPLE_COUNT      5
#define DIST_OFFSET_TIMEOUT_MS        200

#define DIST_SERVO_POS_TOL_MM         8.0f
#define DIST_SERVO_RPM_TOL            60.0f
#define DIST_SERVO_TARGET_CHANGE_TOL_MM 0.5f
#define DIST_SERVO_MAX_SPEED_RPM      9500.0f
#define DIST_SERVO_KP_RPM_PER_MM      34.0f
#define DIST_SERVO_HOLD_KP_RPM_PER_MM 12.0f
#define DIST_SERVO_BRAKE_GAIN_RPM2_PER_MM 50000.0f
#define DIST_SERVO_MIN_MOVE_RPM       140.0f  //最小运动速度
#define DIST_SERVO_APPROACH_SPEED_RPM 220.0f
#define DIST_SERVO_HOLD_MAX_SPEED_RPM 80.0f
#define DIST_SERVO_NEAR_SWITCH_IN_MM  750.0f
#define DIST_SERVO_NEAR_SWITCH_OUT_MM 850.0f
#define DIST_SERVO_ACCEL_RPM_PER_S    22000.0f
#define DIST_SERVO_DECEL_RPM_PER_S    24000.0f
#define DIST_SERVO_DEFAULT_DT_S       0.001f
#define DIST_SERVO_MAX_DT_S           0.020f
#define DIST_SERVO_SENSOR_MIN_MM      20.0f
#define DIST_SERVO_SENSOR_MAX_MM      5000.0f
#define DIST_SERVO_STABLE_MS          200

/* 距离伺服实际有效区间，超出这个范围的测距直接视为无效。 */
#define DIST_SERVO_EFFECTIVE_MIN_MM   100.0f
#define DIST_SERVO_EFFECTIVE_MAX_MM   2800.0f

/* 测距变化速度的物理上限，超过这个斜率就先不让它进入伺服。 */
#define DIST_SERVO_MAX_VALID_SLOPE_MM_PER_S 2000.0f

/* 小于这个差值时，认为串口里还是同一帧测距，不重复更新时间基准。 */
#define DIST_SERVO_NEW_SAMPLE_EPS_MM  1.0f

/* 大跳变出现后，后续样本若仍接近这个候选值，就继续累计观察。 */
#define DIST_SERVO_CANDIDATE_MATCH_MM 80.0f

/* 候选跳变若连续同方向推进，且单步变化不离谱，也允许沿用原候选的起始时间继续累计。 */
#define DIST_SERVO_CONTINUOUS_CANDIDATE_STEP_MM 300.0f

/* 斜率判定用的最大采样间隔上限，防止长时间停更后一次放太大。 */
#define DIST_SERVO_MAX_SAMPLE_DT_S    0.20f
#define DIST_SERVO_INVALID_HOLD_MS    120U
#define DIST_SERVO_RAW_STALE_MS       120U

extern uint16_t distance_offset;

typedef struct {
    float target_distance;
    float raw_distance;
    float reliable_distance;
    float filtered_distance;
    float control_distance;
    float desired_speed_ref;
    float motor_rpm;
    uint8_t use_reliable_near_target;
    uint8_t arrived;
    uint8_t arrived_confirmed;
} DistanceServoDebug_t;

extern DistanceServoDebug_t g_distance_servo_debug;

#define YAW_MAX_SPEED_DEG_PER_S       1220.0f
#define YAW_ACCEL_DEG_PER_S2          950.0f
#define YAW_POS_TOL_DEG               1.0f
#define YAW_ONE_WAY_APPROACH_DEG      6.0f
#define YAW_ONE_WAY_OVERSHOOT_DEG     2.0f
#define YAW_HOLD_KP_DEG_PER_S_PER_DEG  4.0f    /* 到位附近的速度比例系数，越大越“顶” */
#define YAW_MOVE_KP_DEG_PER_S_PER_DEG  14.0f   /* 远离目标时的速度比例系数，越大越激进 */
#define YAW_HOLD_MAX_SPEED_DEG_PER_S   60.0f   /* 到位附近允许的最大速度 */
#define YAW_APPROACH_SPEED_DEG_PER_S   140.0f  /* 接近目标区间的速度上限 */
#define YAW_MIN_MOVE_DEG_PER_S         16.0f   /* 起转最小速度，避免低速抖动不走 */
#define YAW_BRAKE_GAIN_DEG_PER_S2      250.0f  /* 刹车距离估算用的等效减速度 */
#define YAW_STABLE_MS                  120     /* 连续稳定这么久才判定到位 */



#define ARM_MAX_SPEED_DEG_PER_S       1620.0f
#define ARM_ACCEL_DEG_PER_S2          1500.0f
#define ARM_POS_TOL_DEG               1.0f

void YawServo_Reset(void);
uint8_t YawServo_IsArrived(void);
void Yaw_servo(float target_degree, DJI_t *motor);
void ArmServo_Reset(void);
uint8_t ArmServo_IsArrived(void);
void Arm_servo(float target_degree, DJI_t *motor);

void PID_Calc(PID_t *pid);
void P_Calc(PID_t *pid);
void PID_Clear(PID_t *pid);
void positionServo(float ref, DJI_t *motor);
void speedServo(float ref, DJI_t *motor);
float Distance_Speed_Plan(float target_distance, float current_distance, DJI_t *motor);
void DistanceServo_Reset(DJI_t *motor);
uint8_t DistanceServo_IsArrived(DJI_t *motor);
void Distance_servo(float target_distance, DJI_t *motor);
void pr(void);
void clear(void);
void Motor_State_Reset(DJI_t *motor);
void Reset_DJI_Motor_Full(DJI_t *ptr);
void DebugPrint(void);

#endif
