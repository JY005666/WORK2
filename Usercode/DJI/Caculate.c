/*pid算法和速度位置伺服*/

#include "Caculate.h"
#include "math.h"
#include"stdlib.h"
#include "stdint.h"
#include "string.h"

//增量式PID算法
void PID_Calc(PID_t *pid){
	pid->cur_error = pid->ref - pid->fdb;
	pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
	pid->error[0] = pid->error[1];
	pid->error[1] = pid->ref - pid->fdb;
	/*设定输出上限*/
	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
	/*输出死区：输出值太小则直接置零，防止微颤*/
	if(pid->outputMax > 0.0f && fabsf(pid->output) < pid->outputMin)
		pid->output = 0.0f;

}

//比例算法
void P_Calc(PID_t *pid){
	pid->cur_error = pid->ref - pid->fdb;
	pid->output = pid->KP * pid->cur_error;
	/*设定输出上限*/
	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
	
	if(fabs(pid->output)<pid->outputMin)
		pid->output=0;

}

//位置伺服函数
void positionServo(float ref, DJI_t * motor){
	
	motor->posPID.ref = ref;
	motor->posPID.fdb = motor->AxisData.AxisAngle_inDegree;
	PID_Calc(&motor->posPID);
	
	motor->speedPID.ref = motor->posPID.output;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);

}

//速度伺服函数
void speedServo(float ref, DJI_t * motor){
	motor->speedPID.ref = ref;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);
}



/** 
 * @param  target_distance  目标距离（mm），正值远离、负值靠近
 * @param  motor 被控电机指针

 */
void Distance_servo(float target_distance, DJI_t * motor) {
    // ---- 1. 位置环（外环）----
    motor->posPID.ref = target_distance;
    motor->posPID.fdb = lidar.distance_aver;
    PID_Calc(&motor->posPID);

    motor->speedPID.ref = motor->posPID.output;
    motor->speedPID.fdb = motor->FdbData.rpm;
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
    
    // 3. (可选) 如果你需要在代码里记录当前电机的多圈绝对角度，可能需要重新获取一下 offset
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
}



