/**
 * @file DoroboLib32 Trace (DT)
 *
 * DoroboLib32 software based tracing.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#include <stdint.h>
#include <stdbool.h>

#ifndef INCLUDE_TRACE_H_
#define INCLUDE_TRACE_H_


#ifdef __cplusplus
 extern "C" {
#endif


 /**
  * Header for binary trace messages.
  */
typedef struct {
    uint8_t channel; ///< trace channel
    uint8_t sequence_no; ///< sequence number of message (for channel != 32 only)
    uint8_t time_100ms; ///<  timestamp in units of 100 ms
    uint8_t payload_len; ///< length of this trace message (payload only)
} DT_Header_T;


/**
 * @brief   Initialize the tracing module.
 */
void trace_init(void);


/**
 * @brief   Ouput a binary trace message
 *
 * @param channel Trace channel.
 * @param msglen Length of the binary trace message.
 * @param msg Binary trace message.
 */
void trace(uint8_t channel, uint8_t msglen, uint8_t *msg);


/**
 * @brief   Trace a string to channel 32.
 *
 * Trace the given string to trace channel 32. On this channel, no headers
 * (except for the uint8_t channel number) exist. Thus the function effectively
 * prepends a space (' ') and appends a CR ('\r') to the trace message.
 *
 * @param msg Null-terminated C string.
 */
void traces(char *msg);


/**
 * @brief   Trace a printf() style message to channel 32.
 *
 * Formats a printf() style trace message and outputs it to trace channel 32.
 * This function does not support floating point numbers.
 * On this channel, no headers
 * (except for the uint8_t channel number) exist. Thus the function effectively
 * prepends a space (' ') and appends a CR ('\r') to the trace message.
 *
 * @param fmt Null-terminated C format string.
 */
void tracef(char *fmt, ...);


#ifdef __cplusplus
}
#endif


#endif /* INCLUDE_TRACE_H_ */
