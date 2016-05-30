/****************************************************************************
* DroneID handler for UART receive
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
* File: gsm_uart_receive.c
* Purpose: Read UART receive buffer and distribute info to other tasks
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-24 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* system includes */
#include "free_rtos.h"
#include "gsm_uart_receive.h"
#include "Drivers/GLOBAL_QUEUE_FUNCTIONS/global_queue_functions.h"

/***************************************************************************/
/* defines */
#define LONG_TIME 						0xffff
#define UART_GSM_RECEIVE_QUEUE_SIZE 	500
#define GSM_RECEIVE_QUEUE_SIZE 			2000

/***************************************************************************/
// Receive function
void gsm_uart_receive_task(void *arg)
{
	// Create queue to receive data from gsm modem
	xQueue_uart_gsm_receive_handle = xQueueCreate(UART_GSM_RECEIVE_QUEUE_SIZE, sizeof( char ));

	// Protected queue to data received from gsm uart
	xSemaphore_receive_from_gsm_handle = xSemaphoreCreateMutex();
	xQueue_receive_from_gsm_handle = xQueueCreate(GSM_RECEIVE_QUEUE_SIZE, sizeof( char ));

	// Make check if all queues and semaphores are created
	while(!((xSemaphore_virtual_com_send_handle != NULL) && (xQueue_virtual_com_send_handle != NULL)
			&& (xQueue_uart_nmea_receive_handle != NULL) && (xSemaphore_uart_nmea_receive_handle != NULL)))
	{
		vTaskDelay(200/portTICK_RATE_MS);
	}

	// Create byte receive buffer
	char uart_receive_char;
	char uart_receive_buffer[256];
	uint8_t buffer_counter;

	while(true)
	{
		buffer_counter = 0;

		if(xQueueReceive( xQueue_uart_gsm_receive_handle, &uart_receive_char, ( TickType_t ) LONG_TIME ))
		{
			uart_receive_buffer[buffer_counter] = uart_receive_char;
			buffer_counter++;
			uart_receive_buffer[buffer_counter] = NULL;

			// Empty receive buffer
			while(xQueueReceive( xQueue_uart_gsm_receive_handle, &uart_receive_char, ( TickType_t ) 0 ))
			{
				uart_receive_buffer[buffer_counter] = uart_receive_char;
				buffer_counter++;
				uart_receive_buffer[buffer_counter] = NULL;
			}
			add_string_to_queue(uart_receive_buffer, xQueue_receive_from_gsm_handle, 0, xSemaphore_receive_from_gsm_handle, 50);
			add_string_to_queue(uart_receive_buffer, xQueue_uart_nmea_receive_handle, 0, xSemaphore_uart_nmea_receive_handle, 50);
//			add_string_to_queue(uart_receive_buffer, xQueue_virtual_com_send_handle, 0, xSemaphore_virtual_com_send_handle, 50);
		}
		vTaskDelay(5/portTICK_RATE_MS);
	}
}

