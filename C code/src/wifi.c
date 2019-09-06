/**
 * @file DoroboLib32 WIFI (wifi, DW)
 *
 * DoroboLib32 WIFI for the RNXV based on the RN-171 module connected to UART2.
 * Functions for WIFI initialization.
 *
 * Copyright (c) 2016 Michael Hoffmann, Claus Fühner
 */

#include <wifi.h>
#include "stm32f0xx_hal.h"
#include "string.h"
#include "stdlib.h"

#include "trace.h"


#define RX_TIMEOUT 250
#define TX_TIMEOUT 250


#define SERIAL_DEBUG(x, y) \
    HAL_UART_Transmit(&huart1, (x), strlen((x)), 250); \
    HAL_UART_Transmit(&huart1, (y), strlen((y)), 250); \
    HAL_UART_Transmit(&huart1, "\r\n", 2, 250);

// ***************************************************************************

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
static UART_HandleTypeDef *wlan_uart_ptr = &huart2;

#define RX_BUF_SIZE 10
static char rx_buf[RX_BUF_SIZE];

// ***************************************************************************

// TODO Die Eingabe sollte interruptgesteuert oder per DMA erfolgen!!!

static bool get_response(char *expected_answer)
{
    int len = 0;
    HAL_StatusTypeDef receive_retval;
    do {
        // receive a response
        memset(rx_buf, 0, RX_BUF_SIZE);
        receive_retval = HAL_UART_Receive(wlan_uart_ptr, rx_buf, RX_BUF_SIZE, RX_TIMEOUT);

        // try to determine length of answer
        // TODO this cannot really work - fix it!
        len = 0;
        while (len < (RX_BUF_SIZE-1) && rx_buf[len] != '\0')
        {
            len++;
        }
        // make the rx buffer a c string
        rx_buf[len] = '\0';
    } while (len >= (RX_BUF_SIZE-1));
    SERIAL_DEBUG("Received: ", rx_buf);
    SERIAL_DEBUG("Expected: ", expected_answer);
//    itoa(receive_retval, rx_buf, 10);
//    SERIAL_DEBUG("Ret=", rx_buf);

    // check the result
    // TODO wirklichen Vergleich machen
    return true; //(strstr(rx_buf, expected_answer) != NULL);
}

static bool send_command(char *cmd, uint32_t delay_ms, char *expected_answer)
{
    HAL_UART_Transmit(wlan_uart_ptr, cmd, strlen(cmd), TX_TIMEOUT);
    SERIAL_DEBUG("Transmitted: ", cmd);
    HAL_Delay(delay_ms);
    return get_response(expected_answer);
}

static bool send_command3(char *cmd1, char *cmd2, char *cmd3, uint32_t delay_ms, char *expected_answer)
{
    HAL_UART_Transmit(wlan_uart_ptr, cmd1, strlen(cmd1), TX_TIMEOUT);
    SERIAL_DEBUG("Transmitted1: ", cmd1);
    HAL_UART_Transmit(wlan_uart_ptr, cmd2, strlen(cmd2), TX_TIMEOUT);
    SERIAL_DEBUG("Transmitted2: ", cmd2);
    HAL_UART_Transmit(wlan_uart_ptr, cmd3, strlen(cmd3), TX_TIMEOUT);
    SERIAL_DEBUG("Transmitted3: ", cmd3);
    HAL_Delay(delay_ms);
    return get_response(expected_answer);
}

// ***************************************************************************

// TODO Sinnvolle Fehlerbehandlung einführen - z. B. Reset des WIFI-Moduls und Neustart der Prozedur
// TODO WIFI-Modul bis zum Neustart auf schnellere Kommunikationsgeschwindigkeit umschalten und HAL_UART umkonfigurieren

static bool config_start()
{
    bool ok = true;
    // TODO Macht es Sinn, das WIFI-Modul zurückzusetzen?
    ok = ok && send_command("$$$", 200, "CMD");
    ok = ok && send_command("\rload config\r", 0, "AOK");
    HAL_Delay(250);
    return ok;
}

static bool config_leave()
{
    return send_command("\rexit\r", 0, "EXIT");
}

static bool config_ssid(char *ssid, char *passphrase)
{
    bool ok = true;
    ok = ok && send_command3("set wlan ssid ", ssid, "\r", 0, "AOK");
    ok = ok && send_command3("set wlan passphrase ", passphrase, "\r", 0, "AOK");
    ok = ok && send_command("set wlan join 1\r", 0, "AOK");
    ok = ok && send_command("set ip dhcp 1\r", 0, "AOK");
    return ok;
}

static bool config_static_ip(char *ip_address, char *netmask, char *gateway)
{
    bool ok = true;
    ok = ok && send_command("set ip dhcp 0\r", 0, "AOK");
    ok = ok && send_command3("set ip address ", ip_address, "\r", 0, "AOK");
    ok = ok && send_command3("set ip netmask ", netmask, "\r", 0, "AOK");
    ok = ok && send_command3("set ip gateway", gateway, "\r", 0, "AOK");
    return ok;
}

// ***************************************************************************

bool wifi_init()
{
    bool ok = true;
    ok = ok && config_start();
    ok = ok && config_leave();
    return ok;
}

// ***************************************************************************

bool wifi_init_staticip(char *ip_address, char *netmask, char *gateway)
{
    bool ok = true;
    ok = ok && config_start();
    ok = ok && config_static_ip(ip_address, netmask, gateway);
    ok = ok && config_leave();
    return ok;
}

// ***************************************************************************

bool wifi_init_ssid(char *ssid, char *passphrase)
{
    bool ok = true;
    ok = ok && config_start();
    ok = ok && config_ssid(ssid, passphrase);
    ok = ok && config_leave();
    return ok;
}

// ***************************************************************************

bool wifi_init_ssid_staticip(char *ssid, char *passphrase, char *ip_address, char *netmask, char *gateway)
{
    bool ok = true;
    ok = ok && config_start();
    ok = ok && config_ssid(ssid, passphrase);
    ok = ok && config_static_ip(ip_address, netmask, gateway);
    ok = ok && config_leave();
    return ok;
}

