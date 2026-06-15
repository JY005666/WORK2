#ifndef _CACULATE_H__
#define _CACULATE_H__

#include "DJI.h"
#include "stm32f4xx_it.h"
#include "stdio.h"
#include "Stp23L.h"
#include "stm32f4xx_hal.h"  // 使用 HAL_GetTick

// 默认滤波/限幅参数（可根据实测调整）
#define DIST_FILTER_ALPHA       0.03f       // 滤波系数：越小越平滑，应对机构晃动
#define DIST_OFFSET_SAMPLE_COUNT  5
#define DIST_OFFSET_TIMEOUT_MS  200

/*
 * distance 位置伺服速度规划参数
 * 单位说明：
 * - 距离：mm
 * - speedPID.ref：电机转子 rpm，保持与原 speedServo()/Distance_servo() 一致
 * - 加速度/减速度：rpm/s，用于限制速度给定变化率，避免速度给定突变
 *
 * 需要实车微调的主要参数：
 * - DIST_SERVO_MAX_SPEED_RPM：距离轴最高速度给定
 * - DIST_SERVO_ACCEL_RPM_PER_S / DIST_SERVO_DECEL_RPM_PER_S：速度给定斜率
 * - DIST_SERVO_BRAKE_GAIN_RPM2_PER_MM：靠近目标时的刹车曲线强度
 * - DIST_SERVO_MIN_MOVE_RPM：克服摩擦的最小爬行速度
 */
/* 死区大小：机构有晃动时适当加大，避免电机跟着晃 */
#define DIST_SERVO_POS_TOL_MM              5.0f
#define DIST_SERVO_RPM_TOL                 60.0f
#define DIST_SERVO_TARGET_CHANGE_TOL_MM    0.5f
#define DIST_SERVO_MAX_SPEED_RPM           8000.0f
#define DIST_SERVO_KP_RPM_PER_MM           30.0f
#define DIST_SERVO_BRAKE_GAIN_RPM2_PER_MM  25000.0f
#define DIST_SERVO_MIN_MOVE_RPM            100.0f
#define DIST_SERVO_APPROACH_SPEED_RPM      150.0f   // 距目标 50mm 内速度绝对值上限
#define DIST_SERVO_ACCEL_RPM_PER_S         8000.0f
#define DIST_SERVO_DECEL_RPM_PER_S         8000.0f
#define DIST_SERVO_DEFAULT_DT_S            0.001f
#define DIST_SERVO_MAX_DT_S                0.020f
#define DIST_SERVO_SENSOR_MIN_MM           20.0f
#define DIST_SERVO_SENSOR_MAX_MM           5000.0f
/* 到达稳定确认时间（ms）：持续在死区内这么久才标记到达 */
#define DIST_SERVO_STABLE_MS               200


extern uint16_t distance_offset;

// 云台 T 型速度规划参数（可实车微调）
#define YAW_MAX_SPEED_DEG_PER_S    500.0f   // 最大角速度 °/s
#define YAW_ACCEL_DEG_PER_S2       800.0f   // 角加速度 °/s²
#define YAW_POS_TOL_DEG            1.0f     // 到位死区 ±1°

// --- 云台 T 型速度规划接口 ---
void YawServo_Reset(void);
uint8_t YawServo_IsArrived(void);
void Yaw_servo(float target_degree, DJI_t *motor);

// --- PID 与控制接口 ---
void PID_Calc(PID_t *pid);
void P_Calc(PID_t *pid);
void PID_Clear(PID_t *pid);
void positionServo(float ref, DJI_t * motor);
void speedServo(float ref, DJI_t * motor);
float Distance_Speed_Plan(float target_distance, float current_distance, DJI_t *motor);
void DistanceServo_Reset(DJI_t *motor);
uint8_t DistanceServo_IsArrived(DJI_t *motor);
void Distance_servo(float target_distance,DJI_t * motor);
void pr(void);
void clear(void);
void Motor_State_Reset(DJI_t *motor);
void Reset_DJI_Motor_Full(DJI_t *ptr);

#endif



