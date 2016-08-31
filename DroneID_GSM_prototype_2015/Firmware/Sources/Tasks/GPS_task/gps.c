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
#define RECONNECT_GPS_AFTER_MS				10000
#define MAX_MS_BETWEEN_GPGGA_PKG			15000

#define MAX_GPGGA_STR_SIZE					100
#define GPGGA_START_OF_MSG					"GPGGA,"
#define GPGGA_END_OF_MSG					"\r\n"

#define MAX_DISPLACEMENT					3.0
#define MAX_ALTITUDE_DISPLACEMENT			10.0

/***************************************************************************/
// Reset GPGGA shared msg
void reset_gpgga_msg(gpgga_t *gga)
{
	if(xSemaphoreTake(xSemaphore_gga_msg_mutex_handle, 100))
	{
		gga->time[0] = '\0';
		gga->lat = 0;
		gga->lon = 0;
		gga->fix = '\0';
		gga->sat = '\0';
		gga->hdop = 0;
		gga->alt = 0;
		gga->geoid_height = 0;
		gga->dgps_time = 0;
		gga->dgps_stat_id = 0;

		xSemaphoreGive( xSemaphore_gga_msg_mutex_handle );
	}
}

/***************************************************************************/
// Reset GPGGA shared msg
void copy_gga1_to_gga2(gpgga_t *gga1, gpgga_t *gga2)
{
	if(xSemaphoreTake(xSemaphore_gga_msg_mutex_handle, 100))
	{
		int i;
		for(i = 0; i < 11; i++)
		{
			gga2->time[i] = gga1->time[i];
		}
		gga2->lat = gga1->lat;
		gga2->lon = gga1->lon;
		gga2->fix = gga1->fix;
		gga2->sat = gga1->sat;
		gga2->hdop = gga1->hdop;
		gga2->alt = gga1->alt;
		gga2->geoid_height = gga1->geoid_height;
		gga2->dgps_time = gga1->dgps_time;
		gga2->dgps_stat_id = gga1->dgps_stat_id;

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
					reset_gpgga_msg(&gga_msg_global);
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
	// DroneID has moved
	vSemaphoreCreateBinary(xSemaphore_has_moved);

	vSemaphoreCreateBinary(xSemaphore_gps_subscr_deacted);
	vSemaphoreCreateBinary(xSemaphore_gps_subscr_recon);

	xQueue_uart_nmea_receive_handle = xQueueCreate(UART_NMEA_RECEIVE_QUEUE_SIZE, sizeof( char ));
	xSemaphore_uart_nmea_receive_handle = xSemaphoreCreateMutex();

	// Variable for GGA data
	xSemaphore_gga_msg_mutex_handle = xSemaphoreCreateMutex();

	uint32_t msg_delay_timer = 0;

	// Wait until mutex and queues has been created
	while(!(xSemaphore_gpgga_reset != NULL))
	{
		vTaskDelay(200/portTICK_RATE_MS);
	}

	reset_gpgga_msg(&gga_msg_ref);
	reset_gpgga_msg(&gga_msg_global);
	copy_gga1_to_gga2(&gga_msg_global, &gga_msg_ref);

	while(true)
	{
		// If new sequence selected reset buffer
		if(xSemaphoreTake(xSemaphore_gpgga_reset, MS_LOOP_DELAY))
		{
			reset_gpgga_msg(&gga_msg_global);
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

			// Check distance moved
			if((degree_to_meter_conv(&gga_msg_global, &gga_msg_ref) > MAX_DISPLACEMENT) || (gga_msg_ref.alt + MAX_ALTITUDE_DISPLACEMENT < gga_msg_global.alt) || (gga_msg_ref.alt - MAX_ALTITUDE_DISPLACEMENT > gga_msg_global.alt))
			{
				copy_gga1_to_gga2(&gga_msg_global, &gga_msg_ref);
				xSemaphoreGive(xSemaphore_has_moved);
			}
		}
		else
		{
			// If GPS subscriber is disabled pass binary semaphore
			if(msg_delay_timer > MAX_MS_BETWEEN_GPGGA_PKG)
			{
				xSemaphoreGive(xSemaphore_gps_subscr_deacted);
				msg_delay_timer = 0;
			}
			else if(msg_delay_timer > RECONNECT_GPS_AFTER_MS)
			{
				// Send semaphore to reconnect GNSS
				xSemaphoreGive(xSemaphore_gps_subscr_recon);
			}
		}
	}
}
