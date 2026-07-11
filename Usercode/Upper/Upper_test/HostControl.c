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

static uint8_t HostControl_IsValidAxis(uint8_t axis)
{
    return (axis == 0U || axis == 2U || axis == 3U);
}

static uint8_t HostControl_IsValidClaw(uint8_t servo_id)
{
    return (servo_id == 1U || servo_id == 2U);
}

static void HostControl_Send(const char *text)
{
    if (s_host_uart == NULL || text == NULL) return;
    HAL_UART_Transmit(s_host_uart, (uint8_t *)text, (uint16_t)strlen(text), 50);
}

static void HostControl_SendStatus(void)
{
    char msg[256];
    snprintf(msg, sizeof(msg),
             "DIST_TGT=%.2f YAW_TGT=%.2f ARM_TGT=%.2f "
             "CLAW1_TGT=%u CLAW2_TGT=%u "
             "DIST_ACT=%.2f YAW_ACT=%.2f ARM_ACT=%.2f "
             "EN=%u%u%u STOP=%u\r\n",
             g_host_control.target_deg[0],
             g_host_control.target_deg[2],
             g_host_control.target_deg[3],
             g_host_control.claw_target_deg[1],
             g_host_control.claw_target_deg[2],
             lidar.distance_aver,
             hDJI[2].AxisData.AxisAngle_inDegree,
             hDJI[3].AxisData.AxisAngle_inDegree,
             g_host_control.enabled[0],
             g_host_control.enabled[2],
             g_host_control.enabled[3],
             g_host_control.stop_all);
    HostControl_Send(msg);
}

static void HostControl_ZeroAxis(uint8_t axis)
{
    if (!HostControl_IsValidAxis(axis)) return;

    if (axis == 0U) {
        DistanceServo_Reset(&hDJI[0]);
        g_host_control.target_deg[0] = lidar.distance_aver;
        return;
    }

    Reset_DJI_Motor_Full(&hDJI[axis]);
    g_host_control.target_deg[axis] = 0.0f;
}

static void HostControl_SetAxis(uint8_t axis, float degree)
{
    if (!HostControl_IsValidAxis(axis)) return;

    g_host_control.target_deg[axis] = degree;
    g_host_control.enabled[axis] = 1U;
    g_host_control.stop_all = 0U;
}

static void HostControl_SetClaw(uint8_t servo_id, uint8_t degree)
{
    if (!HostControl_IsValidClaw(servo_id)) return;

    g_host_control.claw_target_deg[servo_id] = degree;
    Claw_degree_set(degree, servo_id);
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
    float c1 = 0.0f, c2 = 0.0f;
    char arg1[8] = {0};

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

        if (HostControl_IsValidAxis((uint8_t)axis)) {
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

    if (strcmp(cmd, "CLAW") == 0) {
        cursor += 4;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        axis = strtol(cursor, &endptr, 10);
        if (cursor == endptr) {
            HostControl_Send("ERR claw id\r\n");
            return;
        }

        cursor = endptr;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        deg = strtof(cursor, &endptr);
        if (cursor == endptr) {
            HostControl_Send("ERR claw deg\r\n");
            return;
        }

        if (!HostControl_IsValidClaw((uint8_t)axis)) {
            HostControl_Send("ERR claw id\r\n");
            return;
        }

        if (deg < 0.0f || deg > 180.0f) {
            HostControl_Send("ERR claw range\r\n");
            return;
        }

        HostControl_SetClaw((uint8_t)axis, (uint8_t)deg);
        HostControl_Send("OK\r\n");
        return;
    }

    if (strcmp(cmd, "CLAWALL") == 0) {
        cursor += 7;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        c1 = strtof(cursor, &endptr);
        if (cursor == endptr) {
            HostControl_Send("ERR clawall 1\r\n");
            return;
        }

        cursor = endptr;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        c2 = strtof(cursor, &endptr);
        if (cursor == endptr) {
            HostControl_Send("ERR clawall 2\r\n");
            return;
        }

        if (c1 < 0.0f || c1 > 180.0f || c2 < 0.0f || c2 > 180.0f) {
            HostControl_Send("ERR claw range\r\n");
            return;
        }

        HostControl_SetClaw(1U, (uint8_t)c1);
        HostControl_SetClaw(2U, (uint8_t)c2);
        HostControl_Send("OK\r\n");
        return;
    }

    if (strcmp(cmd, "ZERO") == 0) {
        cursor += 4;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (sscanf(cursor, "%7s", arg1) != 1) {
            HostControl_Send("ERR zero arg\r\n");
            return;
        }

        if (strcmp(arg1, "ALL") == 0) {
            HostControl_ZeroAxis(0);
            HostControl_ZeroAxis(2);
            HostControl_ZeroAxis(3);
            HostControl_Send("OK\r\n");
            return;
        }

        axis = strtol(arg1, &endptr, 10);
        if (arg1 == endptr || !HostControl_IsValidAxis((uint8_t)axis)) {
            HostControl_Send("ERR axis\r\n");
            return;
        }

        HostControl_ZeroAxis((uint8_t)axis);
        HostControl_Send("OK\r\n");
        return;
    }

    if (strcmp(cmd, "STREAM") == 0) {
        cursor += 6;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (sscanf(cursor, "%7s", arg1) != 1) {
            HostControl_Send("ERR stream arg\r\n");
            return;
        }

        if (strcmp(arg1, "ON") == 0) {
            g_host_control.stream_enabled = 1U;
            g_host_control.last_stream_tick = HAL_GetTick();
            HostControl_Send("OK\r\n");
            return;
        }

        if (strcmp(arg1, "OFF") == 0) {
            g_host_control.stream_enabled = 0U;
            HostControl_Send("OK\r\n");
            return;
        }

        HostControl_Send("ERR stream arg\r\n");
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
        HostControl_Send("SET 0 <distance_mm>\r\n");
        HostControl_Send("SET <2|3> <deg>\r\n");
        HostControl_Send("ALL <distance_mm> <deg2> <deg3>\r\n");
        HostControl_Send("CLAW <1|2> <deg>\r\n");
        HostControl_Send("CLAWALL <deg1> <deg2>\r\n");
        HostControl_Send("ZERO <0|2|3|ALL>\r\n");
        HostControl_Send("STREAM <ON|OFF>\r\n");
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
    g_host_control.claw_target_deg[1] = 0U;
    g_host_control.claw_target_deg[2] = 0U;
    g_host_control.stream_period_ms = 100U;
    g_host_control.last_stream_tick = HAL_GetTick();

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
    uint32_t now = HAL_GetTick();

    if (s_host_line_ready) {
        s_host_line_ready = 0U;
        HostControl_HandleLine(s_host_line);
        memset(s_host_line, 0, sizeof(s_host_line));
    }

    if (g_host_control.stream_enabled &&
        (now - g_host_control.last_stream_tick >= g_host_control.stream_period_ms)) {
        g_host_control.last_stream_tick = now;
        HostControl_SendStatus();
    }
}

uint8_t *HostControl_RxBuffer(void)
{
    return &s_host_rx_byte;
}
