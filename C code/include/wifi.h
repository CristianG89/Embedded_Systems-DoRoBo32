/**
 * @file DoroboLib32 WIFI (wifi, DW)
 *
 * DoroboLib32 WIFI for the RNXV based on the RN-171 module connected to UART2.
 * Functions for WIFI initialization.
 *
 * Copyright (c) 2016 Michael Hoffmann, Claus Fühner
 */

#ifndef INCLUDE_WIFI_H_
#define INCLUDE_WIFI_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif


/**
 * @brief   Wlan default initialization for the HW1 Lab using DHCP.
 *
 * @note This function assumes that the WLAN module is connected to its
 * Dorobo32 standard connector (USART2).
 *
 * @return true on success, false otherwise
 */
bool wifi_init();


/**
 * @brief   Wlan default initialization for the HW1 Lab using static IP
 * addresses - use the IP configuration printed on the module.
 *
 * @note This function assumes that the WLAN module is connected to its
 * Dorobo32 standard connector (USART2).
 *
 * @param ip_address IP address of the WLAN module
 * @param netmask Netmask for the module's IP configuration
 * @param gateway Default gateway for the module's IP configuration
 * @return true on success, false otherwise
 */
bool wifi_init_staticip(char *ip_address, char *netmask, char *gateway);


/**
 * @brief   Wlan initialization connect to a given access point with DHCP.
 *
 * @note This function assumes that the WLAN module is connected to its
 * Dorobo32 standard connector (USART2).
 *
 * @param ssid Name (SSID) of the access point to connect to
 * @param passphrase WPA passphrase needed for the connection
 * @return true on success, false otherwise
 */
bool wifi_init_ssid(char *ssid, char *passphrase);


/**
 * @brief   Wlan initialization, connect to the given access point.
 *
 * @note This function assumes that the WLAN module is connected to its
 * Dorobo32 standard connector (USART2).
 *
 * @param ssid Name (SSID) of the access point to connect to
 * @param passphrase WPA passphrase needed for the connection
 * @param ip_address IP address of the WLAN module
 * @param netmask Netmask for the module's IP configuration
 * @param gateway Default gateway for the module's IP configuration
 * @return true on success, false otherwise
 */
bool wifi_init_ssid_staticip(char *ssid, char *passphrase, char *ip_address, char *netmask, char *gateway);


/**
 * @brief   Make the WLAN module an access point with DHCP (softap mode)
 *
 * @note This function assumes that the WLAN module is connected to its
 * Dorobo32 standard connector (USART2).
 *
 * @param ssid Name (SSID) of the access point
 * @param passphrase WPA passphrase needed to connect to the AP
 * @param ip_address Static IP address of the WLAN module
 * @return true on success, false otherwise
 */
bool wifi_init_softap(char *ssid, char *passphrase, char *ip_address);


#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIFI_H_ */
