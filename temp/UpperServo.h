#ifndef __UPPERSERVO_H__
#define __UPPERSERVO_H__

#include "UpperStart.h"
void Upper_Servo_Start(void);

void LightFury_Motor_init();

void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle);
void pid_reset(PID_t *pid, float kp, float ki, float kd);

#endif // __UPPERSERVO_H__