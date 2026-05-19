#ifndef STP23L_H
#define STP23L_H

#include "stm32f4xx_hal.h"
#include "usart.h"

/*STP_23L数据包命令码宏定义*/
#define PACK_GET_DISTANCE       0x02
#define PACK_RESET_SYSTEM       0x0D
#define PACK_STOP               0x0F
#define PACK_ACK                0x10
#define PACK_VERSIOM            0x14


typedef struct{
    int16_t distance;               //测量目标距离（单位：mm）
    float distance_aver;            //为物流赛专门搞的变量，直接计算12个得均值，免得再搞二维数组
    uint16_t noise;                 //环境噪声
    uint32_t peak;                  //接收强度信息
    uint8_t confidence;             //置信度    
    uint32_t intg;                  //积分次数
    int16_t reftof;                 //温度表征值
} LidarPointTypedef;                //获取的测量数据结构体

extern uint8_t Rxbuffer_1[195];//接收缓冲区
extern LidarPointTypedef lidar; 
extern uint8_t UartFlag[1];
/*定义接收到的雷达测试数据*/

void STP23L_Init(UART_HandleTypeDef *huart);
void STP_23L_Decode(uint8_t *buffer, LidarPointTypedef *lidardata);

#endif // __STP_DECODE_H__