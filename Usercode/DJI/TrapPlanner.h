#ifndef TRAPPLANNER_H
#define TRAPPLANNER_H

#include "stm32f4xx_hal.h"
#include "DJI.h"
#include "stp23L.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
//  默认运动参数（可根据实际龙门架调整）
// ============================================================
#define TRAP_MAX_VEL_MMPS       200.0f   // 最大速度 (mm/s)
#define TRAP_ACC_MMPS2          200.0f   // 加速度 (mm/s²)
#define TRAP_STOP_THRESHOLD_MM    1.0f   // 到达判定阈值 (mm)
#define TRAP_ARRIVAL_CNT_THR       5    // 连续多少次在阈值内才算到达

// ============================================================
//  梯形速度规划器状态结构体
// ============================================================
typedef struct {
    float start_pos;         // 起始位置 (mm)
    float target_pos;        // 目标位置 (mm)
    float max_vel;           // 最大速度 (mm/s)
    float acc;               // 加速度 (mm/s²)
    float total_dist;        // 总运动距离 (mm)
    float t_acc;             // 加速段时间 (s)
    float t_const;           // 匀速段时间 (s)
    float t_total;           // 总时间 (s)
    uint32_t start_tick;     // 启动时的系统 tick
    uint8_t active;          // 是否正在运动中
    int arrival_cnt;         // 到达计数器
} TrapPlanner_t;

// ============================================================
//  外部变量声明
// ============================================================
extern LidarPointTypedef lidar;
extern uint16_t distance_offset;
extern CAN_HandleTypeDef hcan1;

// ============================================================
//  PID 函数（Caculate.c 中实现）
// ============================================================
void PID_Calc(PID_t *pid);

// ============================================================
//  规划器接口函数
// ============================================================
void TrapPlanner_Start(TrapPlanner_t *plan, float target_pos);
float TrapPlanner_GetTargetPos(TrapPlanner_t *plan);
float TrapPlanner_GetTargetVel(TrapPlanner_t *plan);
uint8_t TrapPlanner_IsArrived(TrapPlanner_t *plan, float current_pos);
void Distance_servo_WithPlanning(TrapPlanner_t *plan, DJI_t *motor1);

#ifdef __cplusplus
}
#endif

#endif /* TRAPPLANNER_H */

