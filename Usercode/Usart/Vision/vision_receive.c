#include "vision_receive.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>


// 引用 Stp23L.c 中定义的串口接收状态机和字节缓冲
extern RxFrame_t rx_frame;
extern uint8_t rx_byte;

static volatile uint8_t g_done = 0;
static volatile uint8_t g_error = 0;
volatile uint8_t g_vision_ack_done = 0;
uint8_t g_pos[3] = {0};


static void Rx_Reset(void)
{
    memset(&rx_frame, 0, sizeof(rx_frame));
    rx_frame.state = RX_WAIT_HEADER;
    rx_frame.last_tick = HAL_GetTick();
}

static void Frame_Handle(void)
{
    if (rx_frame.cmd != FRAME_CMD)
    {
        g_error = 1;
        return;
    }

    if (rx_frame.len == 0)
    {
        g_error = 1;
        return;
    }

    if (rx_frame.len != 3)
    {
        g_error = 1;
        return;
    }

    g_pos[0] = rx_frame.payload[0];
    g_pos[1] = rx_frame.payload[1];
    g_pos[2] = rx_frame.payload[2];

    g_done = 1;
}

void Vision_Init(void)
{
    Rx_Reset();
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void Vision_Start(uint8_t cmd)
{
    g_done = 0;
    g_error = 0;
    Rx_Reset();

    HAL_UART_Transmit(&huart2, &cmd, 1, 20);
}

void Vision_Process(void)
{
    uint32_t now = HAL_GetTick();

    if (rx_frame.state != RX_WAIT_HEADER)
    {
        if ((now - rx_frame.last_tick) > 80U)
        {
            g_error = 1;
            Rx_Reset();
        }
    }

    if (rx_frame.frame_ready)
    {
        rx_frame.frame_ready = 0;
        Frame_Handle();
    }

    if (rx_frame.frame_error)
    {
        rx_frame.frame_error = 0;
        g_error = 1;
    }
}

uint8_t Vision_IsDone(void)
{
    return g_done;
}

uint8_t Vision_HasError(void)
{
    return g_error;
}

uint8_t Vision_GetPos(uint8_t out[3])
{
    if (!g_done)
        return 0;

    out[0] = g_pos[0];
    out[1] = g_pos[1];
    out[2] = g_pos[2];
    return 1;
}

void Vision_Clear(void)
{
    g_done = 0;
    g_error = 0;
    Rx_Reset();
}

void Vision_ClearAck(void)
{
    g_vision_ack_done = 0;
}

uint8_t Vision_WaitAck(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (g_vision_ack_done)
        {
            g_vision_ack_done = 0;
            return 1;
        }
        osDelay(1);
    }

    return 0;
}

void data_receive(uint8_t *pos_out){
    while (pos_out[0] == 0 || pos_out[1] == 0 || pos_out[2] == 0) 
    {
        Vision_Process();
        if (Vision_IsDone())
            {
            if (Vision_GetPos(pos_out))
            {
                    printf("pos:%d,%d,%d\r\n",pos_out[0],pos_out[1],pos_out[2]);
                    /* 这里拿到的就是：
                    pos_out[0] = 中间豆子应放的绝对位置
                    pos_out[1] = 左侧豆子应放的绝对位置
                
                    pos_out[2] = 右侧豆子应放的绝对位置
                    你队友的机械臂代码直接拿这三个数去调用即可
                    */
            }
            Vision_Clear();
            }
            if (Vision_HasError())
            {
                Vision_Clear();
            }
    }
}
