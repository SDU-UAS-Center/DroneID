/****************************************************************************
* DroneID main state machine
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
* File: drone_id_task.c
* Purpose: Controlling which commands to send to the GSM modem
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-10-17 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* system includes */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

/***************************************************************************/
/* application includes */
#include "free_rtos.h"
#include "queue.h"
#include "Drivers/TYPE_CONVERTER/float_to_string.h"
#include "gpio1.h"
#include "Drivers/SIM808/SIM808_driver.h"
#include "drone_id_task.h"

/***************************************************************************/
/* #defines */
// ID number
#define DRONE_ID					"4"
#define FIRMWARE_VERSION			"1.0"
#define HW_VERSION					"1.0"
// Defines to pkg type
#define FIRMWARE_VERSION_TYPE		"0"
#define RESET_PKG_TYPE				"1"
#define START_TRACKING_PKG_TYPE		"2"
#define UPDATE_WITH_FIX_PKG_TYPE	"3"
#define UPDATE_WITHOUT_FIX_PKG_TYPE	"4"
#define STOP_TRACKING_PKG_TYPE		"5"
#define POWER_OFF_PKG_TYPE			"6"
#define START_NEW_TRIAL_TYPE		"7"
#define USER_DEBUG_PKG_TYPE			"255"
//Seperator between data in pkg
#define SEPERATOR					"\x09"
#define EMPTY_POSITION_CHAR			"\0"

#define MSG_FAILED_TO_SEND			"Error sending tracking msg"
#define RECONNECTED_GNSS			"RECONNECTING GNSS"

// Define msgs to include in pkg to server
#define INCLUDE_TIME				1
#define INCLUDE_LAT					1
#define INCLUDE_LON					1
#define INCLUDE_FIX					0
#define INCLUDE_SAT					1
#define INCLUDE_HDOP				1
#define INCLUDE_ALT					1
#define INCLUDE_GEOID_HEIGHT		1
#define INCLUDE_DGPS_TIME			0
#define INCLUDE_DGPS_STAT_ID		0
#define INCLUDE_TEMP				1
#define INCLUDE_PRESSURE			1
#define INCLUDE_BATT_V				1
#define INCLUDE_RSSI				1
#define INCLUDE_BER					1

// Defines to tracking state machine
#define UPDATE_POS					1
#define STOP_UPDATING_POS			2
#define NEW_TRIAL					3

/***************************************************************************/
/* #types */
enum drone_id_state_types
{
	STOP_TRACKING,
	TRACKING_GPRS
};

enum tracking_states
{
	INIT_GPRS_STATE,
	INIT_GPS_STATE,
	START_TRACKING_STATE,
	SEND_VERSION_STATE,
	TRACKING_STATE
};

/***************************************************************************/
/* function prototypes */
bool create_and_pub_drone_pos(void);
void change_led_indicator_state(lp_state_indicator_types mode);
bool start_gps_and_modem(void);
bool init_gprs(void);
bool enter_license_plate_state_machine(bool first_connect);
void remove_old_semaphore_commands(void);
void send_stop_tracking(void);
void license_plate_modes(void);

/***************************************************************************/
/* Return value of protected double */
double get_protected_double(SemaphoreHandle_t semaphore, uint16_t max_delay, double var)
{
	double return_value = 0.0;
	if(xSemaphoreTake(semaphore, max_delay))
	{
		return_value = var;
		xSemaphoreGive( semaphore );
	}
	return return_value;
}

/***************************************************************************/
/* Create and send an update pkg */
bool create_and_pub_drone_pos(void)
{
	char rssi[3] = {'0','\0','\0'};
	char ber[3] = {'0','\0','\0'};

	if(INCLUDE_RSSI || INCLUDE_BER)
	{
		get_sqr_report(rssi, ber);
		// todo update rssi and ber
	}

	char *gps_pos_msg[40];
	char lat_array[11];
	char lon_array[11];
	char fix_array[3];
	char sat_array[4];
	char hdop_array[6];
	char alt_array[6];
	char geoid_height_array[6];
	char batt_v_array[6];
	char temperature_array[10];
	char pressure_array[10];

	uint8_t msg_pos = 0;
	bool return_value = false;

	if(xSemaphoreTake(xSemaphore_gga_msg_mutex_handle, 5))
	{
		//Add ID
		gps_pos_msg[msg_pos] = DRONE_ID;
		msg_pos++;
		gps_pos_msg[msg_pos] = SEPERATOR;
		msg_pos++;
		// Add package type
		if(gga_msg_global.fix == '\0')
		{
			// Change indicator state
			change_led_indicator_state(TRACKING_NO_FLIGHT_ZONE);
			gps_pos_msg[msg_pos] = UPDATE_WITHOUT_FIX_PKG_TYPE;
			msg_pos++;
			gps_pos_msg[msg_pos] = SEPERATOR;
			msg_pos++;
		}
		else
		{
			// Change indicator state
			change_led_indicator_state(TRACKING_FLIGHT_ZONE);
			gps_pos_msg[msg_pos] = UPDATE_WITH_FIX_PKG_TYPE;
			msg_pos++;
			gps_pos_msg[msg_pos] = SEPERATOR;
			msg_pos++;

			if(INCLUDE_TIME)
			{
				gps_pos_msg[msg_pos] = gga_msg_global.time;
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
			if(INCLUDE_LAT)
			{
				ftoa(gga_msg_global.lat, lat_array, 6);
				gps_pos_msg[msg_pos] = lat_array;
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
			if(INCLUDE_LON)
			{
				ftoa(gga_msg_global.lon, lon_array, 6);
				gps_pos_msg[msg_pos] = lon_array;
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
			if(INCLUDE_FIX)
			{
				snprintf(fix_array, sizeof(fix_array), "%i", gga_msg_global.fix);
				gps_pos_msg[msg_pos] = fix_array;
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
			if(INCLUDE_SAT)
			{
				snprintf(sat_array, sizeof(sat_array), "%i", gga_msg_global.sat);
				gps_pos_msg[msg_pos] = sat_array;
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
			if(INCLUDE_HDOP)
			{
				ftoa(gga_msg_global.hdop, hdop_array, 2);
				gps_pos_msg[msg_pos] = hdop_array;
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
			if(INCLUDE_ALT)
			{
				ftoa(gga_msg_global.alt, alt_array, 1);
				gps_pos_msg[msg_pos] = alt_array;
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
			if(INCLUDE_GEOID_HEIGHT)
			{
				ftoa(gga_msg_global.geoid_height, geoid_height_array, 1);
				gps_pos_msg[msg_pos] = geoid_height_array;
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
			if(INCLUDE_DGPS_TIME)
			{
				// not yet implemented
			}
			if(INCLUDE_DGPS_STAT_ID)
			{
				// not yet implemented
			}
		}

		if(INCLUDE_PRESSURE)
		{
			if(xSemaphoreTake(xSemaphore_pressure_temperature_mutex_handle, 2))
			{
				ftoa(pressure, pressure_array, 2);
				gps_pos_msg[msg_pos] = pressure_array;
				xSemaphoreGive( xSemaphore_pressure_temperature_mutex_handle );
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
		}

		if(INCLUDE_TEMP)
		{
			if(xSemaphoreTake(xSemaphore_pressure_temperature_mutex_handle, 2))
			{
				ftoa(temperature, temperature_array, 2);
				gps_pos_msg[msg_pos] = temperature_array;
				xSemaphoreGive( xSemaphore_pressure_temperature_mutex_handle );
				msg_pos++;
				gps_pos_msg[msg_pos] = SEPERATOR;
				msg_pos++;
			}
		}

		if(INCLUDE_BATT_V)
		{
			if(xSemaphoreTake(xSemaphore_batt_volt_mutex_handle, 2))
			{
				ftoa(batt_voltage, batt_v_array, 2);
				xSemaphoreGive( xSemaphore_batt_volt_mutex_handle );
			}
			gps_pos_msg[msg_pos] = batt_v_array;
			msg_pos++;
			gps_pos_msg[msg_pos] = SEPERATOR;
			msg_pos++;
		}

		if(INCLUDE_RSSI)
		{
			gps_pos_msg[msg_pos] = rssi;
			msg_pos++;
			gps_pos_msg[msg_pos] = SEPERATOR;
			msg_pos++;
		}

		if(INCLUDE_BER)
		{
			gps_pos_msg[msg_pos] = ber;
			msg_pos++;
			gps_pos_msg[msg_pos] = SEPERATOR;
			msg_pos++;
		}

		gps_pos_msg[msg_pos] = EMPTY_POSITION_CHAR;
		// Send data and give back mutex
		return_value = send_msg_to_server(gps_pos_msg);
		xSemaphoreGive( xSemaphore_gga_msg_mutex_handle );
	}
	return return_value;
}



/***************************************************************************/
/* Change LED indicator according to the state og DroneID */
void change_led_indicator_state(lp_state_indicator_types mode)
{
	if(xSemaphoreTake(xSemaphore_lp_state_indicator, 2))
	{
		lp_state_red_indicator = mode;
		xSemaphoreGive( xSemaphore_lp_state_indicator );
	}
}

/***************************************************************************/
/* Init GPRS updater */
bool init_gprs(void)
{
	bool succes = true;

	turn_on_modem_pwr_physical();
	// Maybe this is not needed here
//	reset_gsm_modem();

	// Sync baud and check modem connection
	succes &= send_gsm_sync_msg();

	// Check if GPRS is connected
	if(succes)
	{
		succes &= check_gprs();
	}

	// If no GPRS, don't try the rest
	if(succes)
	{
		// Setup cipmux to single connection
		succes &= setup_single_connection();

		// Setup APN
		succes &= setup_apn();

		// Bring up wireless connection
		succes &= start_wireless_connectin();

		// Get local ip
		succes &= get_local_ip();

		// Connected to server
		succes &= connect_to_udp_server();

		// Enable quick send
		succes &= enable_quick_send_mode();
	}
	return succes;
}

/***************************************************************************/
/* deinit GPRS updater */
bool deinit_gprs(void)
{
	// reset ip
	bool succes = true;
	succes &= reset_ip_session();
	return succes;
}

/***************************************************************************/
/* Init GPS subscriber */
bool init_gps(void)
{
	bool succes = true;
	succes &= power_up_gps();
	succes &= start_publish_nmea_to_uart();
	return succes;
}

/***************************************************************************/
/* Deinit GPS subscriber */
bool deinit_gps(void)
{
	bool succes = true;
	succes &= disable_gps_subscr();
	succes &= disable_gps_pwr();
	return succes;
}

/***************************************************************************/
/* Empty semaphore msgs */
void remove_old_semaphore_commands(void)
{
	xSemaphoreTake(xSemaphore_pre_state, 0);
	xSemaphoreTake(xSemaphore_next_state, 0);
}

/***************************************************************************/
/* Inform server of start or stop tracking pkg */
bool send_start_stop_tracking(char *pkg_type)
{
	char voltage_buf[6] = "0.0";
	ftoa(get_protected_double(xSemaphore_batt_volt_mutex_handle, 2, batt_voltage), voltage_buf, 2);
	char *msg[6] = {DRONE_ID, SEPERATOR, pkg_type, SEPERATOR, voltage_buf, "\0"};
	// Send msg

	return send_msg_to_server(msg);
}

/***************************************************************************/
/* Send debug msg to server */
void send_debug_msg(char *error_msg)
{
	char *msg[6] = {DRONE_ID, SEPERATOR, USER_DEBUG_PKG_TYPE, SEPERATOR, error_msg, "\0"};
	send_msg_to_server(msg);
}

/***************************************************************************/
/* Send msg to server and receive confirmation from server */
bool send_firmware_version()
{
	char *msg[8] = {DRONE_ID, SEPERATOR, FIRMWARE_VERSION_TYPE, SEPERATOR, HW_VERSION, SEPERATOR, FIRMWARE_VERSION, "\0"};
	return send_msg_receive_server_resond(msg);
}

/***************************************************************************/
/*  */
void gprs_tracking_state_machine(uint8_t update_mode)
{
	static enum tracking_states cur_tracking_state = INIT_GPRS_STATE;
	static uint8_t error_counter = 0;

	switch (cur_tracking_state)
	{
	case INIT_GPRS_STATE:
		change_led_indicator_state(MODULE_CONNECTING_GPRS);
		turn_off_modem_pwr_physical();
		vTaskDelay(500/portTICK_RATE_MS);

		if(init_gprs())
		{
			cur_tracking_state = INIT_GPS_STATE;
		}
		else
		{
			reset_gsm_modem();
			cur_tracking_state = INIT_GPRS_STATE;
		}
		break;

	case INIT_GPS_STATE:
		change_led_indicator_state(MODULE_CONNECTING_GPS);
		if(init_gps())
		{
			// Use when sending nmea to usb only
//			while(true)
//			{
//				vTaskDelay(20000/portTICK_RATE_MS);
//			}
			cur_tracking_state = SEND_VERSION_STATE;
			// Delete old data from gpgga buffer
			xSemaphoreGive(xSemaphore_gpgga_reset);
		}
		else
		{
			reset_gsm_modem();
			cur_tracking_state = INIT_GPRS_STATE;
		}
		break;

	case SEND_VERSION_STATE:
		if(send_firmware_version())
		{
			cur_tracking_state = START_TRACKING_STATE;
			error_counter = 0;
		}
		else
		{
			close_udp_msg();
			error_counter++;
			if(error_counter > 3)
			{
				reset_gsm_modem();
				cur_tracking_state = INIT_GPRS_STATE;
			}
		}
		break;

	case START_TRACKING_STATE:
		if(send_start_stop_tracking(START_TRACKING_PKG_TYPE))
		{
			cur_tracking_state = TRACKING_STATE;
			error_counter = 0;
		}
		else
		{
			close_udp_msg();
			error_counter++;
			if(error_counter > 3)
			{
				reset_gsm_modem();
				cur_tracking_state = INIT_GPRS_STATE;
			}
		}
		break;

	case TRACKING_STATE:
		if(xSemaphoreTake(xSemaphore_next_state, 0))
		{
			send_start_stop_tracking(STOP_TRACKING_PKG_TYPE);
			vTaskDelay(100/portTICK_RATE_MS);
			send_start_stop_tracking(START_NEW_TRIAL_TYPE);
			cur_tracking_state = START_TRACKING_STATE;
		}
		// Check if GPS subscriber is active
		else if(xSemaphoreTake(xSemaphore_gps_subscr_deacted, 0))
		{
			reset_gsm_modem();
			cur_tracking_state = INIT_GPRS_STATE;
		}
		else if(xSemaphoreTake(xSemaphore_gps_subscr_recon, 0))
		{
			// De init gnss
			deinit_gps();
			vTaskDelay(10/portTICK_RATE_MS);
			// Reconnect GNSS
			init_gps();
			send_debug_msg(RECONNECTED_GNSS);
			xSemaphoreTake(xSemaphore_gps_subscr_recon, 0);
		}
		// Send tracking msg
		else if((xSemaphoreTake(xSemaphore_has_moved, 0) || (xSemaphoreTake(xSemaphore_send_udp, 0))))
		{
			if(create_and_pub_drone_pos())
			{
				xSemaphoreGive(xSemaphore_send_timer_reset);
				cur_tracking_state = TRACKING_STATE;
				error_counter = 0;
			}
			else
			{
				vTaskDelay(200/portTICK_RATE_MS);
				close_udp_msg();
				send_debug_msg(MSG_FAILED_TO_SEND);
				error_counter++;
				if(error_counter >= 3)
				{
					reset_gsm_modem();
					cur_tracking_state = INIT_GPRS_STATE;
				}
			}
		}
		break;
	}

	// Stop gsm modem
	if(update_mode == STOP_UPDATING_POS)
	{
		send_start_stop_tracking(STOP_TRACKING_PKG_TYPE);
		vTaskDelay(200/portTICK_RATE_MS);

		turn_off_modem_pwr_physical();
		cur_tracking_state = INIT_GPRS_STATE;
	}
}

/***************************************************************************/
/* State machine for DroneIDs states */
void enter_drone_id_states(void)
{
	enum drone_id_state_types drone_id_state = STOP_TRACKING;
	while(true)
	{
		switch (drone_id_state)
		{
		case STOP_TRACKING:
			// Change indicator state
			change_led_indicator_state(MODULE_POWERED_OFF);

			if(xSemaphoreTake(xSemaphore_droneid_mode_selector, 5000))
			{
				if (droneid_mode_selector == POWER_ON)
				{
					change_led_indicator_state(MODULE_CONNECTING_GPRS);
					drone_id_state = TRACKING_GPRS;
				}
				xSemaphoreGive(xSemaphore_droneid_mode_selector);
				remove_old_semaphore_commands();
			}


			break;


		case TRACKING_GPRS:
			xSemaphoreGive(xSemaphore_get_new_voltage_sample);
			// Request new battery voltage mess
			vTaskDelay(20/portTICK_RATE_MS);

			bool stop_tracking_flag = false;

			if(xSemaphoreTake(xSemaphore_droneid_mode_selector, 0))
			{
				if (droneid_mode_selector == POWER_OFF)
				{
					stop_tracking_flag = true;
					change_led_indicator_state(MODULE_POWERED_OFF);
					gprs_tracking_state_machine(STOP_UPDATING_POS);
					drone_id_state = STOP_TRACKING;
				}
				xSemaphoreGive(xSemaphore_droneid_mode_selector);
			}

			if(!stop_tracking_flag)
			{
					gprs_tracking_state_machine(UPDATE_POS);
			}
			break;
		}
		vTaskDelay(10/portTICK_RATE_MS);
	}
}

/***************************************************************************/
/* DroneID main task */
void drone_id_main_task(void *arg)
{
	// Reset gpgga data semaphore
	vSemaphoreCreateBinary(xSemaphore_gpgga_reset);

	vSemaphoreCreateBinary(xSemaphore_send_timer_reset);

	// Wait until mutex and queues has been created
	while(!((xSemaphore_pre_state != NULL) && (xSemaphore_next_state != NULL) &&
			(xQueue_uart_gsm_send_handle != NULL) && (xSemaphore_uart_gsm_send_handle != NULL) &&
			(xSemaphore_pressure_temperature_mutex_handle != NULL) && (xSemaphore_send_udp != NULL)&&
			(xSemaphore_gps_subscr_deacted != NULL) && (xSemaphore_get_new_voltage_sample != NULL) &&
			(xSemaphore_gga_msg_mutex_handle != NULL) && (xSemaphore_has_moved != NULL) &&
			(xSemaphore_gps_subscr_recon!= NULL)))
	{
		vTaskDelay(200/portTICK_RATE_MS);
	}
	remove_old_semaphore_commands();

	// Enter while loop
	enter_drone_id_states();
}

