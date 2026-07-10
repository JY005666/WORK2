#ifndef HOSTCONTROL_H
#define HOSTCONTROL_H

#include "main.h"
#include "usart.h"
#include "DJI.h"
#include "Caculate.h"

typedef struct
{
    float target_deg[4];
    uint8_t enabled[4];
    uint8_t stop_all;
    uint8_t stream_enabled;
    uint32_t stream_period_ms;
    uint32_t last_stream_tick;
} HostControlState_t;

extern HostControlState_t g_host_control;

void HostControl_Start(UART_HandleTypeDef *huart);
void HostControl_OnByteReceived(uint8_t byte);
void HostControl_Process(void);
void HostControl_StopAll(void);
uint8_t *HostControl_RxBuffer(void);

#endif
