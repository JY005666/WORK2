#ifndef _CACULATE_H__
#define _CACULATE_H__

#include "DJI.h"
#include"stm32f4xx_it.h"
#include"stdio.h"
#include"stp23L.h"
#include "stm32f4xx_hal.h"  // 使用 HAL_GetTick

// 默认滤波/限幅参数（可根据实测调整）
#define DIST_FILTER_ALPHA       0.08f
#define DIST_OFFSET_SAMPLE_COUNT  5
#define DIST_OFFSET_TIMEOUT_MS  200

extern uint16_t distance_offset;

// --- PID 与控制接口 ---
void PID_Calc(PID_t *pid);
void P_Calc(PID_t *pid);
void positionServo(float ref, DJI_t * motor);
void speedServo(float ref, DJI_t * motor);
void Distance_servo(float target_distance,DJI_t * motor);
void pr(void);
void clear(void);
void Motor_State_Reset(DJI_t *motor);
void Reset_DJI_Motor_Full(DJI_t *ptr);

#endif