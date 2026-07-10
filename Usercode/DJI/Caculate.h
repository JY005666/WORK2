#ifndef _CACULATE_H__
#define _CACULATE_H__

#include "DJI.h"
#include "stm32f4xx_it.h"
#include "stdio.h"
#include "Stp23L.h"
#include "stm32f4xx_hal.h"

#define DIST_FILTER_ALPHA             0.03f
#define DIST_OFFSET_SAMPLE_COUNT      5
#define DIST_OFFSET_TIMEOUT_MS        200

#define DIST_SERVO_POS_TOL_MM         5.0f
#define DIST_SERVO_RPM_TOL            60.0f
#define DIST_SERVO_TARGET_CHANGE_TOL_MM 0.5f
#define DIST_SERVO_MAX_SPEED_RPM      9500.0f
#define DIST_SERVO_KP_RPM_PER_MM      34.0f
#define DIST_SERVO_BRAKE_GAIN_RPM2_PER_MM 32000.0f
#define DIST_SERVO_MIN_MOVE_RPM       140.0f
#define DIST_SERVO_APPROACH_SPEED_RPM 220.0f
#define DIST_SERVO_ACCEL_RPM_PER_S    12000.0f
#define DIST_SERVO_DECEL_RPM_PER_S    14000.0f
#define DIST_SERVO_DEFAULT_DT_S       0.001f
#define DIST_SERVO_MAX_DT_S           0.020f
#define DIST_SERVO_SENSOR_MIN_MM      20.0f
#define DIST_SERVO_SENSOR_MAX_MM      5000.0f
#define DIST_SERVO_STABLE_MS          200

extern uint16_t distance_offset;

#define YAW_MAX_SPEED_DEG_PER_S       700.0f
#define YAW_ACCEL_DEG_PER_S2          1200.0f
#define YAW_POS_TOL_DEG               1.0f

void YawServo_Reset(void);
uint8_t YawServo_IsArrived(void);
void Yaw_servo(float target_degree, DJI_t *motor);

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
