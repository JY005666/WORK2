#include "vision_sync.h"
#include "usart.h"
#include <string.h>

#define START_CMD       0x11

#define FRAME_HEADER    0xAA
#define FRAME_CMD       0x31
#define FRAME_TAIL      0x55
#define MAX_PAYLOAD_LEN 3

typedef enum
{
    RX_WAIT_HEADER = 0,
    RX_WAIT_CMD,
    RX_WAIT_LEN,
    RX_WAIT_PAYLOAD,
    RX_WAIT_CRC,
    RX_WAIT_TAIL
} RxState_t;

typedef struct
{
    RxState_t state;
    uint8_t len;
    uint8_t idx;
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t cmd;
    uint8_t crc_rx;
    volatile uint8_t frame_ready;
    volatile uint8_t frame_error;
    uint32_t last_tick;
} RxFrame_t;

static uint8_t rx_byte;
static RxFrame_t rx_frame;

static volatile uint8_t g_done = 0;
static volatile uint8_t g_error = 0;
static uint8_t g_pos[3] = {0};

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

void Vision_Start(void)
{
    uint8_t cmd = START_CMD;

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

/* 串口中断接收回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
        return;

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