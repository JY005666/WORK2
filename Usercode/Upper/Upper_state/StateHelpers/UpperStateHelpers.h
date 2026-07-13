#ifndef UPPER_STATE_HELPERS_H
#define UPPER_STATE_HELPERS_H

#include "../UpperState.h"

#define CLAW_UP 1
#define CLAW_DOWN 2
#define CLAW_OPEN 120
#define CLAW_HALF_OPEN 90
#define CLAW_CLOSE 50

#define SAFE_DIST_FOR_BOX_FINAL_TURN_MM  1500.0f
#define FIRST_BEAN_LEFT_SAFE_CCW_DEG     (-80.0f)
#define FIRST_BEAN_RIGHT_SAFE_CW_DEG     (60.0f)
#define SECOND_BEAN_FINAL_TURN_MM        1500.0f
#define SECOND_BEAN_LEFT_TO_RIGHT_AVOID_CCW_DEG   (60.0f)
#define SECOND_BEAN_RIGHT_TO_LEFT_AVOID_CW_DEG    (-60.0f)

#define LEFT_BEAN_DELIVERY_LIFT_TARGET_DEG     (-650.0f)
#define LEFT_BEAN_DELIVERY_LIFT_READY_DEG      (-400.0f)
#define RIGHT_BEAN_DELIVERY_LIFT_TARGET_DEG    (-650.0f)
#define RIGHT_BEAN_DELIVERY_LIFT_READY_DEG     (-600.0f)
#define MIDDLE_BEAN_DELIVERY_LIFT_TARGET_DEG   (-650.0f)
#define MIDDLE_BEAN_DELIVERY_LIFT_READY_DEG    (-500.0f)

uint8_t ApplyDeliveryLiftGate(BeanPosition bean_position);
void SetApproachTarget(const Angle *target);
void PrepareBeanPickup(const Angle *target, float claw_hold_degree, float arm_ready_threshold);
uint8_t IsDistanceAndChassisReady(float distance_tol, float chassis_tol);
void ResetDistanceAndChassisMotors(void);
void CloseClawAndAdvance(uint16_t next_stage);
void LiftAndRotateToPlacement(float lift_target, float lift_ready_threshold, float chassis_target, float chassis_tol, uint16_t next_stage);
void HandleStage30_FirstBeanPlacement(void);
void HandleStage31_SecondBeanPickup(void);
void HandleActiveBeanDelivery(uint16_t next_stage_left, uint16_t next_stage_right);
void HandleStage900_ThirdBeanPickup(void);
void Bean_Place_Switch(Bean *b, uint16_t next_stage);

#endif
