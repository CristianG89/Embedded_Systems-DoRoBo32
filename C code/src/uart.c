/**
 * DoroboLib32 UART (DUART)
 *
 * Functions for UART communication.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#include "stm32f0xx_hal.h"
#include "string.h"

#include "uart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static UART_HandleTypeDef* select_uart(enum DS_UART_E euart);

void uart_init(void)
{
    ;
}

void uart_send(enum DS_UART_E euart, char* msg)
{
    UART_HandleTypeDef *current_huart = &huart1;
    current_huart = select_uart(euart);
    HAL_UART_Transmit(current_huart, msg, strlen(msg), 500);
}

void uart_send_buffer(enum DS_UART_E uart, uint8_t *pbuffer, uint16_t size)
{
    UART_HandleTypeDef *current_huart = &huart1;
    current_huart = select_uart(uart);
    HAL_UART_Transmit(current_huart, pbuffer, size, 500);
}

void uart_receive(enum DS_UART_E euart, uint8_t* pbuffer, uint16_t size)
{
    UART_HandleTypeDef *current_huart = &huart1;
    current_huart = select_uart(euart);
    HAL_UART_Receive(current_huart, pbuffer, size, 200);
}

static UART_HandleTypeDef* select_uart(enum DS_UART_E euart)
{
    UART_HandleTypeDef *current_huart = &huart1;
    switch (euart)
    {
    case DS_UART1:
    {
        current_huart = &huart1;
        break;
    }
    case DS_UART2:
    {
        current_huart = &huart2;
        break;
    }
    default:
        current_huart = &huart1;
        break;
    }
    return current_huart;
}

//void USART1_IRQHandler(void)
//{
//    HAL_UART_IRQHandler(&huart1);
//}
//
//void USART2_IRQHandler(void)
//{
//    HAL_UART_IRQHandler(&huart2);
//}
