/****************************************************************************
* DroneID tracking timer
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
* File: tracking_send_timer_task.c
* Purpose: Pass binary semaphore to send new pkg
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-11-26 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* system includes */

/***************************************************************************/
/* application includes */
#include "free_rtos.h"
#include "tracking_timer_task.h"

/***************************************************************************/
/* #defines */
#define MS_BEFORE_SEND_NEW_PKG				1000

/***************************************************************************/
/* function prototypes */
void create_binary_semaphore_to_send_udp(void);
void enter_timer_function(void);

/***************************************************************************/
/* Create binary semaphore to pass signals to other tasks */
void create_binary_semaphore_to_send_udp(void)
{
	// Create semaphores to tap through DroneID states
	vSemaphoreCreateBinary(xSemaphore_send_udp);
}

/***************************************************************************/
/* Pass a binary semaphore for every loop itt.*/
void enter_timer_function(void)
{
	while(true)
	{
		xSemaphoreGive(xSemaphore_send_udp);
		vTaskDelay(MS_BEFORE_SEND_NEW_PKG/portTICK_RATE_MS);
	}
}

/***************************************************************************/
/* Send msg to send udp task*/
void tracking_timer_task(void *arg)
{
	create_binary_semaphore_to_send_udp();
	vTaskDelay(1000/portTICK_RATE_MS);
	enter_timer_function();
}
/***************************************************************************/
