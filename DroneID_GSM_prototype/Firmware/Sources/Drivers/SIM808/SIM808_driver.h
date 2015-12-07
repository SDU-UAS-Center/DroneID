/****************************************************************************
* SIM808driver functions
* Copyright (c) 2015, Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*    * Neither the name of the copyright holder nor the names of its
*      contributors may be used to endorse or promote products derived from
*      this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
* File: SIM808_driver.h
* Purpose: Functions for gsm modem SIM808
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-22 Martin Skriver, Source written
****************************************************************************/

#ifndef SIM808_DRIVER_H_
#define SIM808_DRIVER_H_

/***************************************************************************/
/* Include external variables */
// Protected queue to data send to GSM from gsm send
extern QueueHandle_t xQueue_uart_gsm_send_handle;
extern SemaphoreHandle_t xSemaphore_uart_gsm_send_handle;

// Protected queue to data received from gsm uart
extern SemaphoreHandle_t xSemaphore_receive_from_gsm_handle;
extern QueueHandle_t xQueue_receive_from_gsm_handle;

// Protected queue to data send via virtual com
extern SemaphoreHandle_t xSemaphore_virtual_com_send_handle;
extern QueueHandle_t xQueue_virtual_com_send_handle;

/***************************************************************************/
/* Shared functins */
// Power on gsm modem
void turn_on_modem_pwr_physical(void);

// Power off gsm modem
void turn_off_modem_pwr_physical(void);

// Reset gsm modem
void reset_gsm_modem(void);

// Set ports for init gsm modem ready for receive via uart
void init_gsm_modem(void);

// Sync with gsm modem and check connection
bool send_gsm_sync_msg(void);

// Enable gps unit and start finding satellites
bool power_up_gps(void);

// Check if gprs is attached.
bool check_gprs(void);

// Setup cipmux to single connection
bool setup_single_connection(void);

// Setup APN according to defined user
bool setup_apn(void);

// This will bring up the wireless connection
bool start_wireless_connectin(void);

// This gets the local IP address TODO read if error
bool get_local_ip(void);

// Connect to server
bool connect_to_udp_server(void);

// Start quick send
bool enable_quick_send_mode(void);

// Send msg to socket server
bool send_msg_to_server(char **msg);

// Start publish NMEA to UART
bool start_publish_nmea_to_uart(void);

// Reset IP session
bool reset_ip_session(void);

// Close connection
bool close_connection(void);

// Stop gps subscriber
bool disable_gps_subscr(void);

// Power off gps
bool disable_gps_pwr(void);

// Close udp msg
bool close_udp_msg(void);

#endif /* SIM808_DRIVER_H_ */
