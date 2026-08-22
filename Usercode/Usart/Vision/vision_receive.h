#ifndef __VISION_SYNC_H
#define __VISION_SYNC_H

#include "main.h"
#include <stdint.h>

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

extern uint8_t g_pos[3];
extern volatile uint8_t g_vision_ack_done;

void Vision_Init(void);
void Vision_Start(uint8_t cmd);
void Vision_Process(void);

uint8_t Vision_IsDone(void);
uint8_t Vision_HasError(void);
uint8_t Vision_GetPos(uint8_t out[3]);

void data_receive(uint8_t *pos_out);     // 改为带参数，传入外部数组指针
void Vision_Clear(void);
void Vision_ClearAck(void);
uint8_t Vision_WaitAck(uint32_t timeout_ms);

#endif

