#include"Stp23L.h"
#include <string.h>

// ---------- 接收缓冲区及状态机变量 ----------
uint8_t Rxbuffer_1[195];

LidarPointTypedef lidar;

uint8_t UartFlag[1] = {0};

uint8_t usart1_rx[1];

void STP23L_Init(UART_HandleTypeDef *huart) {
    // 启动单字节接收中断
    HAL_UART_Receive_IT(huart, usart1_rx, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        static uint16_t u1state = 0; //状态机计数
        static uint16_t crc1    = 0; //校验和
        uint8_t tmp1   = usart1_rx[0];
        if (u1state < 4)
        {
            if (tmp1 == 0xAA)
            {
                Rxbuffer_1[u1state] = tmp1;
                u1state++;
                
            } else {
                u1state = 0;
            }
        } else if (u1state < 194) {
            Rxbuffer_1[u1state] = tmp1;
            u1state++;
            crc1 += tmp1;
        }else if(u1state==194){
            Rxbuffer_1[u1state] = tmp1;
            if (tmp1 == crc1 % 256) 
            {
                UartFlag[0] = 1;
            }
            u1state = 0;
            crc1    = 0;
        } else {
        };

        HAL_UART_Receive_IT(&huart1, usart1_rx, 1);
    }
}
void STP_23L_Decode(uint8_t *buffer, LidarPointTypedef*lidardata) // num:指明是第几个雷达，本代码框架中范围为0-3
{
    if((buffer[0]==buffer[1])&&(buffer[1]==buffer[2])&&(buffer[2]==buffer[3])&&(buffer[3]==0xAA))     //检测帧头
    {
        if (buffer[5] == PACK_GET_DISTANCE)                                                                       //检测命令码
         {   uint32_t CS_sum = 0;
            for (uint16_t i = 4; i < 194; i++) CS_sum += buffer[i];
            if (buffer[194] == CS_sum%256)                                                                        //检测校验码
            {
                float dis_sum = 0;
                for (uint16_t i = 0; i < 12; i++) {
                    lidardata->distance = buffer[10 + 15 * i] + (buffer[11 + 15 * i] << 8); // 只有distance是有用的数据 注意前面是低位后面是高位
                    dis_sum += lidardata->distance;
                    // lidardata->noise      = (buffer[13 + 15 * i] << 8) + buffer[12 + 15 * i];
                    // lidardata->peak       = (buffer[17 + 15 * i] << 24) + (buffer[15 + 15 * i] << 16) + (buffer[16 + 15 * i] << 8) + buffer[15 + 15 * i];
                    // lidardata->confidence = buffer[18 + 15 * i];
                    // lidardata->intg       = (buffer[22 + 15 * i] << 24) + (buffer[21 + 15 * i] << 16) + (buffer[20 + 15 * i] << 8) + buffer[19 + 15 * i];
                    // lidardata->reftof     = (buffer[24 + 15 * i] << 8) + buffer[23 + 15 * i];
                }
                lidardata->distance_aver = dis_sum / 12;
            }
         }
    }
}