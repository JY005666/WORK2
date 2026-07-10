#include "HostControl.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define HOST_RX_LINE_MAX 64

HostControlState_t g_host_control = {0};

static UART_HandleTypeDef *s_host_uart = NULL;
static uint8_t s_host_rx_byte = 0;
static char s_host_line[HOST_RX_LINE_MAX];
static uint8_t s_host_line_len = 0;
static volatile uint8_t s_host_line_ready = 0;

static void HostControl_Send(const char *text)
{
    if (s_host_uart == NULL || text == NULL) return;
    HAL_UART_Transmit(s_host_uart, (uint8_t *)text, (uint16_t)strlen(text), 50);
}

static void HostControl_SendStatus(void)
{
    char msg[160];
    snprintf(msg, sizeof(msg),
             "TARGET[0]=%.2f TARGET[2]=%.2f TARGET[3]=%.2f "
             "ACT[0]=%.2f ACT[2]=%.2f ACT[3]=%.2f "
             "EN=%u%u%u STOP=%u\r\n",
             g_host_control.target_deg[0],
             g_host_control.target_deg[2],
             g_host_control.target_deg[3],
             hDJI[0].AxisData.AxisAngle_inDegree,
             hDJI[2].AxisData.AxisAngle_inDegree,
             hDJI[3].AxisData.AxisAngle_inDegree,
             g_host_control.enabled[0],
             g_host_control.enabled[2],
             g_host_control.enabled[3],
             g_host_control.stop_all);
    HostControl_Send(msg);
}

static void HostControl_SetAxis(uint8_t axis, float degree)
{
    if (axis >= 4U) return;

    g_host_control.target_deg[axis] = degree;
    g_host_control.enabled[axis] = 1U;
    g_host_control.stop_all = 0U;
}

void HostControl_StopAll(void)
{
    memset(g_host_control.enabled, 0, sizeof(g_host_control.enabled));
    g_host_control.stop_all = 1U;

    PID_Clear(&hDJI[0].posPID);
    PID_Clear(&hDJI[0].speedPID);
    PID_Clear(&hDJI[2].posPID);
    PID_Clear(&hDJI[2].speedPID);
    PID_Clear(&hDJI[3].posPID);
    PID_Clear(&hDJI[3].speedPID);
}

static void HostControl_HandleLine(char *line)
{
    char cmd[8] = {0};
    char *cursor = line;
    char *endptr = NULL;
    long axis = 0;
    float deg = 0.0f;
    float d0 = 0.0f, d2 = 0.0f, d3 = 0.0f;

    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }

    if (sscanf(cursor, "%7s", cmd) != 1) {
        HostControl_Send("ERR empty\r\n");
        return;
    }

    if (strcmp(cmd, "SET") == 0) {
        cursor += 3;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        axis = strtol(cursor, &endptr, 10);
        if (cursor == endptr) {
            HostControl_Send("ERR set axis\r\n");
            return;
        }
        cursor = endptr;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        deg = strtof(cursor, &endptr);
        if (cursor == endptr) {
            HostControl_Send("ERR set deg\r\n");
            return;
        }

        if (axis == 0 || axis == 2 || axis == 3) {
            HostControl_SetAxis((uint8_t)axis, deg);
            HostControl_Send("OK\r\n");
        } else {
            HostControl_Send("ERR axis\r\n");
        }
        return;
    }

    if (strcmp(cmd, "ALL") == 0) {
        cursor += 3;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        d0 = strtof(cursor, &endptr);
        if (cursor == endptr) {
            HostControl_Send("ERR all 0\r\n");
            return;
        }
        cursor = endptr;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        d2 = strtof(cursor, &endptr);
        if (cursor == endptr) {
            HostControl_Send("ERR all 2\r\n");
            return;
        }
        cursor = endptr;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        d3 = strtof(cursor, &endptr);
        if (cursor == endptr) {
            HostControl_Send("ERR all 3\r\n");
            return;
        }

        HostControl_SetAxis(0, d0);
        HostControl_SetAxis(2, d2);
        HostControl_SetAxis(3, d3);
        HostControl_Send("OK\r\n");
        return;
    }

    if (strcmp(cmd, "STOP") == 0) {
        HostControl_StopAll();
        HostControl_Send("OK\r\n");
        return;
    }

    if (strcmp(cmd, "GET") == 0) {
        HostControl_SendStatus();
        return;
    }

    if (strcmp(cmd, "HELP") == 0) {
        HostControl_Send("SET <0|2|3> <deg>\r\n");
        HostControl_Send("ALL <deg0> <deg2> <deg3>\r\n");
        HostControl_Send("GET\r\nSTOP\r\n");
        return;
    }

    HostControl_Send("ERR cmd\r\n");
}

void HostControl_Start(UART_HandleTypeDef *huart)
{
    s_host_uart = huart;
    s_host_line_len = 0U;
    s_host_line_ready = 0U;
    memset(s_host_line, 0, sizeof(s_host_line));
    memset(&g_host_control, 0, sizeof(g_host_control));
    g_host_control.stop_all = 1U;

    HAL_UART_Receive_IT(s_host_uart, &s_host_rx_byte, 1);
    HostControl_Send("HOST CTRL READY\r\n");
    HostControl_Send("DEFAULT STOP\r\n");
}

void HostControl_OnByteReceived(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (s_host_line_len > 0U) {
            s_host_line[s_host_line_len] = '\0';
            s_host_line_ready = 1U;
            s_host_line_len = 0U;
        }
    } else if (s_host_line_len < (HOST_RX_LINE_MAX - 1U)) {
        s_host_line[s_host_line_len++] = (char)byte;
    } else {
        s_host_line_len = 0U;
    }

    HAL_UART_Receive_IT(s_host_uart, &s_host_rx_byte, 1);
}

void HostControl_Process(void)
{
    if (!s_host_line_ready) return;

    s_host_line_ready = 0U;
    HostControl_HandleLine(s_host_line);
    memset(s_host_line, 0, sizeof(s_host_line));
}

uint8_t *HostControl_RxBuffer(void)
{
    return &s_host_rx_byte;
}
