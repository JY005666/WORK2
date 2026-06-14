

#include "UpperServo.h"


Gantry_COMPONENT LightFury;    //两个分区的龙门的数据



/********线程相关部分*************/

void Upper_Servo_Task(void *argument)
{
    osDelay(100);
    for (;;) {

        positionServo_lidar(LightFury.gantry_t.position.x, LightFury.Motor_X, Lidar1);
    
        positionServo_lidar(LightFury.gantry_t.position.yL, LightFury.Motor_YL, Lidar2);
        
        positionServo_lidar(LightFury.gantry_t.position.yR, LightFury.Motor_YR, Lidar3);
        

        // positionServo_lidar(3500,  LightFury.Motor_X,  Lidar1);
        // positionServo_lidar(current_posYL, LightFury.Motor_YL, Lidar2);
        // positionServo_lidar(current_posYR, LightFury.Motor_YR, Lidar3);

        CanTransmit_DJI_1234(&hcan1,
                             LightFury.Motor_X->speedPID.output,
                             -1 * LightFury.Motor_X->speedPID.output,
                             LightFury.Motor_YL->speedPID.output,
                             LightFury.Motor_YR->speedPID.output);
        CanTransmit_DJI_5678(&hcan1,
                             0,
                             0,
                             0,
                             0);

        osDelay(2);
    }
    
}

void Upper_Servo_Start(void)
{
    osThreadId_t Upper_ServoHandle;
    const osThreadAttr_t Upper_Servo_attributes = {
        .name       = "Upper_Servo",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    Upper_ServoHandle = osThreadNew(Upper_Servo_Task, NULL, &Upper_Servo_attributes);
}

/*******封装函数部分********/
void LightFury_Motor_init()               //电机初始化
{
    
    LightFury.Motor_X  = &hDJI[0];
    LightFury.Motor_YL = &hDJI[2];
    LightFury.Motor_YR = &hDJI[3];
    
    hDJI[0].motorType  = M2006;
    hDJI[1].motorType  = M2006;
    hDJI[2].motorType  = M2006;
   
    DJI_Init();
    pid_reset(&(LightFury.Motor_X->speedPID), 3, 0.3, 0.3);
    pid_reset(&(LightFury.Motor_YL->speedPID),5, 0.4, 0.3);
    pid_reset(&(LightFury.Motor_YR->speedPID),5, 0.4, 0.3);
    
   

    pid_reset(&(LightFury.Motor_X->posPID), -30, 0, 0);
    pid_reset(&(LightFury.Motor_YL->posPID), -30, 0, 0);
    pid_reset(&(LightFury.Motor_YR->posPID), 30, 0, 0);
    

    LightFury.Motor_X->posPID.outputMax = 20000;
    LightFury.Motor_YL->posPID.outputMax = 20000;
    LightFury.Motor_YR->posPID.outputMax = 20000;
    CANFilterInit(&hcan1);
}




/**
 * @brief T型速度规划函数
 * @param initialAngle 初始角度
 * @param maxAngularVelocity 最大角速度
 * @param AngularAcceleration 角加速度
 * @param targetAngle 目标角度
 * @param currentTime 当前时间
 * @param currentAngle 当前角度
 * @todo 转换为国际单位制
 */
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle)
{

    float angleDifference = targetAngle - initialAngle;     // 计算到目标位置的角度差
    float sign            = (angleDifference > 0) ? 1 : -1; // 判断角度差的正负(方向)

    float accelerationTime = maxAngularVelocity / AngularAcceleration;                                                      // 加速(减速)总时间
    float constTime        = (fabs(angleDifference) - AngularAcceleration * pow(accelerationTime, 2)) / maxAngularVelocity; // 匀速总时间
    float totalTime        = constTime + accelerationTime * 2;                                                              // 计算到达目标位置所需的总时间

    // 判断能否达到最大速度
    if (constTime > 0) {
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= accelerationTime + constTime) {
            // 匀速阶段
            *currentAngle = initialAngle + sign * maxAngularVelocity * (currentTime - accelerationTime) + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime - constTime;
            *currentAngle          = initialAngle + sign * maxAngularVelocity * constTime + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
        }
    } else {
        maxAngularVelocity = sqrt(fabs(angleDifference) * AngularAcceleration);
        accelerationTime   = maxAngularVelocity / AngularAcceleration;
        totalTime          = 2 * accelerationTime;
        constTime          = 0;
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime; // 减速时间
            *currentAngle          = initialAngle + sign * 0.5 * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
           
        }
    }
}

void pid_reset(PID_t *pid, float kp, float ki, float kd)
{
    pid->KP       = kp;
    pid->KI       = ki;
    pid->KD       = kd;
    // pid->integral = 0;
    // pid->output   = 0;
}