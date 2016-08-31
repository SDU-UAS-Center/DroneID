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
* File: gps.h
* Purpose: Check for new received GPGGA msg, parse and publish it for other tasks.
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-22 Martin Skriver, Source written
****************************************************************************/

#ifndef GPS_H_
#define GPS_H_

/***************************************************************************/
/* application includes */
#include "free_rtos.h"
#include "Tasks/GSM_uart_task/gsm_uart_send.h"
#include "Drivers/NMEA/nmea.h"

/***************************************************************************/
/* Include external variables */
// GGA msg and mutex
SemaphoreHandle_t xSemaphore_gga_msg_mutex_handle;
gpgga_t gga_msg_global;
gpgga_t gga_msg_ref;

// Binary semaphore to reset gpgga data buffer
extern xSemaphoreHandle xSemaphore_gpgga_reset;

/***************************************************************************/
/* Shared variables */
TaskHandle_t GPS_task_handle;

// Protected queue to pass received data to gps task
QueueHandle_t xQueue_uart_nmea_receive_handle;
SemaphoreHandle_t xSemaphore_uart_nmea_receive_handle;

// Binary semaphore indicate if gps subscriber is inactive
xSemaphoreHandle xSemaphore_gps_subscr_deacted;
xSemaphoreHandle xSemaphore_gps_subscr_recon;

// DroneID has moved binary
xSemaphoreHandle xSemaphore_has_moved;


/***************************************************************************/
/* shared functions */
void gps_task(void *arg);

#endif /* GPS_H_ */
