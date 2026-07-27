#ifndef _CACULATE_H__
#define _CACULATE_H__

#include "DJI.h"
#include "Stp23L.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stdio.h"

#define DIST_FILTER_ALPHA             1.0f
#define DIST_OFFSET_SAMPLE_COUNT      5
#define DIST_OFFSET_TIMEOUT_MS        200

#define DIST_SERVO_POS_TOL_MM         8.0f
#define DIST_SERVO_RPM_TOL            60.0f
#define DIST_SERVO_TARGET_CHANGE_TOL_MM 0.5f
#define DIST_SERVO_MAX_SPEED_RPM      9500.0f
#define DIST_SERVO_KP_RPM_PER_MM      44.0f
#define DIST_SERVO_HOLD_KP_RPM_PER_MM 12.0f
#define DIST_SERVO_BRAKE_GAIN_RPM2_PER_MM 60000.0f
#define DIST_SERVO_MIN_MOVE_RPM       140.0f  //最小运动速度
#define DIST_SERVO_APPROACH_SPEED_RPM 220.0f
#define DIST_SERVO_HOLD_MAX_SPEED_RPM 80.0f

#define DIST_SERVO_NEAR_SWITCH_IN_MM  1050.0f
#define DIST_SERVO_NEAR_SWITCH_OUT_MM 1250.0f

#define DIST_SERVO_ACCEL_RPM_PER_S    35000.0f
#define DIST_SERVO_DECEL_RPM_PER_S    50000.0f
#define DIST_SERVO_DEFAULT_DT_S       0.001f
#define DIST_SERVO_MAX_DT_S           0.020f
#define DIST_SERVO_SENSOR_MIN_MM      20.0f
#define DIST_SERVO_SENSOR_MAX_MM      5000.0f
#define DIST_SERVO_STABLE_MS          200

/* 距离伺服实际有效区间，超出这个范围的测距直接视为无效。 */
#define DIST_SERVO_EFFECTIVE_MIN_MM   100.0f
#define DIST_SERVO_EFFECTIVE_MAX_MM   3000.0f

#define DIST_SERVO_INVALID_HOLD_MS    120U
#define DIST_SERVO_INVALID_HOLD_RPM   800.0f

extern uint16_t distance_offset;

typedef struct {
    float target_distance;
    float live_distance;
    float raw_distance;
    float reliable_distance;
    float filtered_distance;
    float control_distance;
    float error_distance;
    float stop_error_distance;
    float desired_speed_ref;
    float motor_rpm;
    uint8_t use_reliable_near_target;
    uint8_t arrived;
    uint8_t arrived_confirmed;
    uint8_t stall_limited;
    uint8_t mismatch_braking;
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
#define YAW_HOLD_ZERO_TOL_DEG          0.4f    /* 到位附近小于该误差时直接停住，避免末端来回抖 */



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
void YawServo_ForceLockCurrent(DJI_t *motor);

#endif
