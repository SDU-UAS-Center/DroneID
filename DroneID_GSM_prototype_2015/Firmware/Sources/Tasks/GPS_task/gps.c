/****************************************************************************
* GPGGA msg publisher
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
* File: gps.c
* Purpose: Check for new received GPGGA msg, parse and publish it for other tasks.
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-22 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* application includes */
#include "free_rtos.h"
#include "queue.h"
#include "gps.h"
#include "Drivers/GLOBAL_QUEUE_FUNCTIONS/global_queue_functions.h"

/***************************************************************************/
/* #defines */
#define UART_NMEA_RECEIVE_QUEUE_SIZE		2000

#define MS_LOOP_DELAY						500
#define MAX_MS_BETWEEN_GPGGA_PKG			15000

#define MAX_GPGGA_STR_SIZE					100
#define GPGGA_START_OF_MSG					"GPGGA,"
#define GPGGA_END_OF_MSG					"\r\n"

/***************************************************************************/
// Reset GPGGA shared msg
void reset_gpgga_msg(void)
{
	if(xSemaphoreTake(xSemaphore_gga_msg_mutex_handle, 100))
	{
		gga_msg_global.time[0] = '\0';
		gga_msg_global.lat = 0;
		gga_msg_global.lon = 0;
		gga_msg_global.fix = '\0';
		gga_msg_global.sat = '\0';
		gga_msg_global.hdop = 0;
		gga_msg_global.alt = 0;
		gga_msg_global.geoid_height = 0;
		gga_msg_global.dgps_time = 0;
		gga_msg_global.dgps_stat_id = 0;

		xSemaphoreGive( xSemaphore_gga_msg_mutex_handle );
	}
}

/***************************************************************************/
// If gga msg received do validate else return
bool update_gpgga_data(void)
{
	bool return_value = false;

	char gga_buffer[MAX_GPGGA_STR_SIZE];
	if(return_string_with_identifier(xQueue_uart_nmea_receive_handle, gga_buffer, GPGGA_START_OF_MSG, GPGGA_END_OF_MSG, MAX_GPGGA_STR_SIZE, 10))
	{
		if(!nmea_checksum(gga_buffer))
		{
			if(xSemaphoreTake(xSemaphore_gga_msg_mutex_handle, 200))
			{
				if(nmea_gpgga_parse(gga_buffer, &gga_msg_global))
				{
					reset_gpgga_msg();
				}
				else
				{
					return_value = true;
				}
				xSemaphoreGive( xSemaphore_gga_msg_mutex_handle );
			}
		}
	}
	return return_value;
}

/***************************************************************************/
// Check if new gpgga msg is received and publish to shared variable
void gps_task(void *arg)
{
	vSemaphoreCreateBinary(xSemaphore_gps_subscr_deacted);

	xQueue_uart_nmea_receive_handle = xQueueCreate(UART_NMEA_RECEIVE_QUEUE_SIZE, sizeof( char ));
	xSemaphore_uart_nmea_receive_handle = xSemaphoreCreateMutex();

	uint32_t msg_delay_timer = 0;

	// Wait until mutex and queues has been created
	while(!(xSemaphore_gpgga_reset != NULL))
	{
		vTaskDelay(200/portTICK_RATE_MS);
	}


	while(true)
	{
		// If new sequence selected reset buffer
		if(xSemaphoreTake(xSemaphore_gpgga_reset, MS_LOOP_DELAY))
		{
			reset_gpgga_msg();
			msg_delay_timer = 0;
			clear_queue(xQueue_uart_nmea_receive_handle, xSemaphore_uart_nmea_receive_handle, 10);
			xSemaphoreTake(xSemaphore_gps_subscr_deacted, 0);
		}

		msg_delay_timer += MS_LOOP_DELAY;

		// Check if new GPGGA msg is received
		if(update_gpgga_data())
		{
			msg_delay_timer = 0;
			xSemaphoreTake(xSemaphore_gps_subscr_deacted, 0);
		}
		else
		{
			// If GPS subscriber is disabled pass binary semaphore
			if(msg_delay_timer > MAX_MS_BETWEEN_GPGGA_PKG)
			{
				xSemaphoreGive(xSemaphore_gps_subscr_deacted);
				msg_delay_timer = 0;
			}
		}
	}
}
