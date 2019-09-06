/**
 * @file
 *
 * DoroboLib32 USART (DS)
 *
 * Functions for UART communication.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#ifndef DOROBO32_INC_UART_H_
#define DOROBO32_INC_UART_H_

#include <stdint.h>

/**
 * Dorobo32 uarts
 */
enum DS_UART_E
{
    DS_UART1,    ///< UART1 connected to the FTDI USB Serial converter
    DS_UART2     ///< UART2 connected to the WLAN/Zigbee module connector
};

#ifdef __cplusplus
 extern "C" {
#endif

 /**
 * @brief   UART initialization.
 */
void uart_init(void);

/**
 * @brief   Send string via UART.
 *
 * @param   uart
 *          The UART instance to be used as defined in DUART_UART_E
 *
 * @param   string
 *          char* string to be transmitted
 */
void uart_send(enum DS_UART_E uart, char* string);

/**
 * @brief   Send buffer via UART.
 *
 * @param   uart
 *          The UART instance to be used as defined in DUART_UART_E
 *
 * @param   *pbuffer
 *          Pointer to buffer
 *
 * @param   size
 *          Number of character to be sent
 */
void uart_send_buffer(enum DS_UART_E uart, uint8_t *pbuffer, uint16_t size);

/**
 * @brief   Receive messages from UART
 *
 * @param   uart
 *          UART instance to be used as defined in DUART_UART_E
 *
 * @param   *pbuffer
 *          Buffer to store received data
 *
 * @param   size
 *          Number of bytes to be received
 */
void uart_receive(enum DS_UART_E uart, uint8_t* pbuffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* DOROBO32_INC_UART_H_ */
