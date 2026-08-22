#include"Stp23L.h"
#include "HostControl.h"
#include <string.h>

extern volatile uint8_t g_vision_ack_done;




uint8_t rx_byte;
RxFrame_t rx_frame;

static volatile uint8_t g_done = 0;
static volatile uint8_t g_error = 0;

static uint8_t CalcCrc8(const uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFF);
}

static void Rx_Reset(void)
{
    memset(&rx_frame, 0, sizeof(rx_frame));
    rx_frame.state = RX_WAIT_HEADER;
    rx_frame.last_tick = HAL_GetTick();
}





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
    if(huart->Instance == USART2){
        static uint8_t ack_state = 0;

        switch (ack_state)
        {
            case 0:
                ack_state = (rx_byte == 0xAA) ? 1U : 0U;
                break;

            case 1:
                ack_state = (rx_byte == 0x32) ? 2U : ((rx_byte == 0xAA) ? 1U : 0U);
                break;

            case 2:
                ack_state = (rx_byte == 0x11) ? 3U : ((rx_byte == 0xAA) ? 1U : 0U);
                break;

            case 3:
                if (rx_byte == 0x55)
                {
                    g_vision_ack_done = 1U;
                }
                ack_state = (rx_byte == 0xAA) ? 1U : 0U;
                break;

            default:
                ack_state = 0U;
                break;
        }

        rx_frame.last_tick = HAL_GetTick();
        switch (rx_frame.state)
        {
            case RX_WAIT_HEADER:
                if (rx_byte == FRAME_HEADER)
                    rx_frame.state = RX_WAIT_CMD;
                break;

            case RX_WAIT_CMD:
                rx_frame.cmd = rx_byte;
                rx_frame.state = RX_WAIT_LEN;
                break;

            case RX_WAIT_LEN:
                if (rx_byte <= MAX_PAYLOAD_LEN)
                {
                    rx_frame.len = rx_byte;
                    rx_frame.idx = 0;

                    if (rx_frame.len == 0)
                        rx_frame.state = RX_WAIT_CRC;
                    else
                        rx_frame.state = RX_WAIT_PAYLOAD;
                }
                else
                {
                    Rx_Reset();
                }
                break;

            case RX_WAIT_PAYLOAD:
                rx_frame.payload[rx_frame.idx++] = rx_byte;
                if (rx_frame.idx >= rx_frame.len)
                    rx_frame.state = RX_WAIT_CRC;
                break;

            case RX_WAIT_CRC:
            {
                rx_frame.crc_rx = rx_byte;
                uint8_t buf[2 + MAX_PAYLOAD_LEN];
                uint8_t used = 0;

                buf[used++] = rx_frame.cmd;
                buf[used++] = rx_frame.len;
                for (uint8_t i = 0; i < rx_frame.len; i++)
                    buf[used++] = rx_frame.payload[i];

                if (CalcCrc8(buf, used) == rx_frame.crc_rx)
                    rx_frame.state = RX_WAIT_TAIL;
                else
                    rx_frame.frame_error = 1;
                break;
            }

            case RX_WAIT_TAIL:
                if (rx_byte == FRAME_TAIL)
                    rx_frame.frame_ready = 1;
                else
                    rx_frame.frame_error = 1;

                rx_frame.state = RX_WAIT_HEADER;
                break;

                    default:
                Rx_Reset();
                break;
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
    if (huart->Instance == UART4) {
        HostControl_OnByteReceived(*HostControl_RxBuffer());
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
