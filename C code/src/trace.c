/**
 * @file DoroboLib32 Trace (DT)
 *
 * DoroboLib32 software based tracing.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */


#include "trace.h"

#include "stm32f0xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"


// ***************************************************************************

extern UART_HandleTypeDef huart2;
static UART_HandleTypeDef *trace_uart_ptr = &huart2;

#define TRACE_BUFLEN 80
static char trace_buf[TRACE_BUFLEN];

// ***************************************************************************

void trace_init(void)
{
    // nothing to do
}

// ***************************************************************************

void trace(uint8_t channel, uint8_t msglen, uint8_t *msg)
{
    static uint8_t sequence;
    DT_Header_T header;

    header.channel = channel;
    header.sequence_no = sequence;
    header.time_100ms = HAL_GetTick() / 100;
    header.payload_len = msglen;
    HAL_UART_Transmit(trace_uart_ptr, (uint8_t *)&header, sizeof(header), 20);
    HAL_UART_Transmit(trace_uart_ptr, msg, sizeof(msglen), 200);
}

// ***************************************************************************

void traces(char *msg)
{
    HAL_UART_Transmit(trace_uart_ptr, " ", 1, 100);
    HAL_UART_Transmit(trace_uart_ptr, msg, strlen(msg), 100);
    HAL_UART_Transmit(trace_uart_ptr, "\n", 1, 100);
}

void tracef(char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsniprintf(trace_buf, TRACE_BUFLEN, fmt, args);
    traces(trace_buf);
    va_end(args);
}
