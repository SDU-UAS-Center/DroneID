/****************************************************************************
 * SIM808 driver functions
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
 * File: SIM808_driver.c
 * Purpose: Functions for gsm modem SIM808
 * Project: DroneID
 * Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
 * ****************************************************************************
 * Log:
 * Created:  2015-09-22 Martin Skriver, Source written
 ****************************************************************************/

/***************************************************************************/
/* application includes */
#include "gpio1.h"
#include "Drivers/GLOBAL_QUEUE_FUNCTIONS/global_queue_functions.h"
#include "Tasks/GSM_uart_task/gsm_uart_send.h"
#include "SIM808_driver.h"

/***************************************************************************/
/* defines */
// General AT msgs
#define AT_GENERAL_OK_RESPOND				"OK\r\n"

// Defines for sync task
#define AT_SYNC_COMMAND						"AT\n"
#define SYNC_ITTERATIONS					200

// Defines power up gps
#define AT_GNSS_PWR_EN_COMMAND				"AT+CGNSPWR=1\n"

// Defines for gprs is attached
#define AT_GPRS_STATUS_COMMAND				"AT+CGATT?\n"
#define AT_GPRS_STATUS_RESPOND				"+CGATT: 1\r\n"
#define MAX_GPRS_ITTERATIONS				300

// Defines for setup cipmux
#define AT_SINGLE_CONNECTION_COMMAND		"AT+CIPMUX=0\n"

// Defines for APN function
#define AT_APN_USER_PASS_COMMAND			"AT+CSTT=\"internet\",\"\",\"\"\n"

// Defines til bring up wireless connection
#define AT_BRING_UP_WIRELESS_COMMAND		"AT+CIICR\n"

// Defines to get local ip address
#define AT_GET_IP_COMMAND					"AT+CIFSR\n"

// Defines to connect to server
#define AT_CONNECT_TO_SERVER_COMMAND		"AT+CIPSTART=\"UDP\",\"diam.uaslab.dk\",\"5050\"\n"
#define AT_CONNECTED_OK_RESPOND				"CONNECT OK\r\n"

// Defines to enable quick send
#define AT_ENABLE_Q_SEND_COMMAND			"AT+CIPQSEND=1\n"

// Defines to open a new msg
#define AT_UDP_SEND_COMMAND					"AT+CIPSEND\n"
#define AT_UDP_SEND_RESPOND					"> "
#define END_OF_MSG_COMMAND					" \x1A"
#define AT_UDP_SEND_OK_RESPOND				"DATA ACCEPT:"

// Defines to publish nmea to uart
#define AT_SEND_NMEA_TO_UART_COMMAND		"AT+CGNSTST=1\n"

// Defines to close connection
#define AT_CLOSE_CONNECTION_COMMAND			"AT+CIPCLOSE\n"

// Defines to reset ip sesion
#define AT_RESET_IP_SESSION_COMMAND			"AT+CIPSHUT\n"

// Stop gps subscr
#define AT_STOP_GPS_SUBSCR_COMMAND			"AT+CGNSTST=0\n"

// Power off gps
#define AT_POWER_OFF_GPS_COMMAND			"AT+CGNSPWR=0\n"

/***************************************************************************/
// Power on gsm modem
void turn_on_modem_pwr_physical(void)
{
	GPIO_DRV_SetPinOutput(SIM808_PWRKEY);
	GPIO_DRV_SetPinOutput(SIM808_RESET);
	vTaskDelay(4000 / portTICK_RATE_MS);
}

/***************************************************************************/
// Power off gsm modem
void turn_off_modem_pwr_physical(void)
{
	GPIO_DRV_ClearPinOutput(SIM808_PWRKEY);
	GPIO_DRV_ClearPinOutput(SIM808_RESET);
}

/***************************************************************************/
// Reset gsm modem
void reset_gsm_modem(void)
{
	// Reset GSM module min 105mS delay
	GPIO_DRV_ClearPinOutput(SIM808_RESET);
	vTaskDelay(200 / portTICK_RATE_MS);
	GPIO_DRV_SetPinOutput(SIM808_RESET);
	vTaskDelay(1000 / portTICK_RATE_MS);
}

/***************************************************************************/
// General send sequence
bool send_gsm_command(char *data_to_send, uint16_t queue_block_time, uint16_t semaphore_block_time, uint16_t ms_before_compare, char **respond, uint16_t respond_timeout)
{
	clear_queue(xQueue_receive_from_gsm_handle, xSemaphore_receive_from_gsm_handle, semaphore_block_time);
	add_string_to_queue(data_to_send, xQueue_uart_gsm_send_handle, queue_block_time, xSemaphore_uart_gsm_send_handle, semaphore_block_time);
	vTaskDelay(ms_before_compare / portTICK_RATE_MS);
	return queue_find_compare_match(respond, xQueue_receive_from_gsm_handle, respond_timeout);
}

/***************************************************************************/
// Sync with gsm modem and check connection
bool send_gsm_sync_msg(void)
{
	bool return_value = false;
	uint8_t sync_counter;

	// Buffers for AT commands
	char send_command[sizeof(AT_SYNC_COMMAND)] = AT_SYNC_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };

	// Make sync sequence until responds accepted
	for (sync_counter = 0;((sync_counter < SYNC_ITTERATIONS) && (!return_value));sync_counter++)
	{
		vTaskDelay(100 / portTICK_RATE_MS);
		return_value = send_gsm_command(send_command, 50, 50, 100, pointer_array, 100);
	}
	return return_value;
}

/***************************************************************************/
// Enable gps unit and start finding satellites
bool power_up_gps(void)
{
	char send_command[sizeof(AT_GNSS_PWR_EN_COMMAND)] =	AT_GNSS_PWR_EN_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 100, pointer_array, 5000);
}

/***************************************************************************/
// Check if gprs is attached.
bool check_gprs(void)
{
	bool return_value = false;
	char send_command[sizeof(AT_GPRS_STATUS_COMMAND)] = AT_GPRS_STATUS_COMMAND;
	char *pointer_array[3] = { AT_GPRS_STATUS_RESPOND, AT_GENERAL_OK_RESPOND };

	uint8_t counter;

	for (counter = 0; ((!return_value) && (counter < MAX_GPRS_ITTERATIONS)); counter++)
	{
		return_value = send_gsm_command(send_command, 50, 50, 400, pointer_array, 100);
	}
	return return_value;
}

/***************************************************************************/
// Setup cipmux to single connection
bool setup_single_connection(void)
{
	char send_command[sizeof(AT_SINGLE_CONNECTION_COMMAND)] = AT_SINGLE_CONNECTION_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 100, pointer_array, 5000);
}

/***************************************************************************/
// Setup APN according to defined user
bool setup_apn(void)
{
	char send_command[sizeof(AT_APN_USER_PASS_COMMAND)] = AT_APN_USER_PASS_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 100, pointer_array, 5000);
}

/***************************************************************************/
// This will bring up the wireless connection
bool start_wireless_connectin(void)
{
	char send_command[sizeof(AT_BRING_UP_WIRELESS_COMMAND)] = AT_BRING_UP_WIRELESS_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 100, pointer_array, 10000);
}

/***************************************************************************/
// This gets the local IP address TODO read if error
bool get_local_ip(void)
{
	bool return_value = false;
	char send_command[sizeof(AT_GET_IP_COMMAND)] = AT_GET_IP_COMMAND;
	return_value = clear_queue(xQueue_receive_from_gsm_handle, xSemaphore_receive_from_gsm_handle, 200);
	// Send AT command
	if (return_value)
	{
		return_value = add_string_to_queue(send_command, xQueue_uart_gsm_send_handle, 50, xSemaphore_uart_gsm_send_handle, 50);
	}
	vTaskDelay(500 / portTICK_RATE_MS);
	return return_value;
}

/***************************************************************************/
// Connect to server
bool connect_to_udp_server(void)
{
	char send_command[sizeof(AT_CONNECT_TO_SERVER_COMMAND)] = AT_CONNECT_TO_SERVER_COMMAND;
	char *pointer_array[3] = { AT_GENERAL_OK_RESPOND, AT_CONNECTED_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 200, pointer_array, 3000);
}

/***************************************************************************/
// Start quick send
bool enable_quick_send_mode(void)
{
	char send_command[sizeof(AT_ENABLE_Q_SEND_COMMAND)] = AT_ENABLE_Q_SEND_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 200, pointer_array, 5000);
}

bool close_udp_msg(void)
{
	char *send_respond_ptr[2] = { AT_UDP_SEND_OK_RESPOND };
	return send_gsm_command(END_OF_MSG_COMMAND, 100, 100, 10, send_respond_ptr, 500);
}
/***************************************************************************/
// Send msg to socket server
bool send_msg_to_server(char **msg)
{
	bool return_value = false;
	char send_command[sizeof(AT_UDP_SEND_COMMAND)] = AT_UDP_SEND_COMMAND;
	char *pointer_array[2] = { AT_UDP_SEND_RESPOND };
	return_value = send_gsm_command(send_command, 100, 100, 20, pointer_array, 300);
	vTaskDelay(20/portTICK_RATE_MS);

	return_value |= add_array_of_strings_to_queue(msg, xQueue_uart_gsm_send_handle, 100, xSemaphore_uart_gsm_send_handle, 100);
	vTaskDelay(20/portTICK_RATE_MS);
	return_value |= close_udp_msg();

	return return_value;
}

/***************************************************************************/
// Start publish NMEA to UART
bool start_publish_nmea_to_uart(void)
{
	char send_command[sizeof(AT_SEND_NMEA_TO_UART_COMMAND)] = AT_SEND_NMEA_TO_UART_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 100, pointer_array, 1500);
}

/***************************************************************************/
// Close connection
bool close_connection(void)
{
	char send_command[sizeof(AT_CLOSE_CONNECTION_COMMAND)] = AT_CLOSE_CONNECTION_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 100, pointer_array, 500);
}

/***************************************************************************/
// Reset IP session
bool reset_ip_session(void)
{
	char send_command[sizeof(AT_RESET_IP_SESSION_COMMAND)] =  AT_RESET_IP_SESSION_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 500, pointer_array, 10000);
}

/***************************************************************************/
// Stop gps subscriber
bool disable_gps_subscr(void)
{
	char send_command[sizeof(AT_STOP_GPS_SUBSCR_COMMAND)] = AT_STOP_GPS_SUBSCR_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 500, pointer_array, 2000);
}

/***************************************************************************/
// Power off gps
bool disable_gps_pwr(void)
{
	char send_command[sizeof(AT_POWER_OFF_GPS_COMMAND)] = AT_POWER_OFF_GPS_COMMAND;
	char *pointer_array[2] = { AT_GENERAL_OK_RESPOND };
	return send_gsm_command(send_command, 100, 50, 500, pointer_array, 2000);
}
