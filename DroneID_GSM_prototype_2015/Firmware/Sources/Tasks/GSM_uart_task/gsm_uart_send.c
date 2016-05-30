/****************************************************************************
* DroneID handler for UART send
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
* File: gsm_uart_send.c
* Purpose: Add bytes from send queue to UART send buffer
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-24 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* system includes */
#include "free_rtos.h"
#include "lpuartCom1.h"
#include "gsm_uart_send.h"

/***************************************************************************/
/* #defines */
#define LONG_TIME 0xffff
#define UART_GSM_SEND_QUEUE_SIZE 1000

/***************************************************************************/
// Send function
void gsm_uart_send_task(void *arg)
{
	// Create protected queue to send data to gsm modem
	xSemaphore_uart_gsm_send_handle = xSemaphoreCreateMutex();
	xQueue_uart_gsm_send_handle = xQueueCreate(UART_GSM_SEND_QUEUE_SIZE, sizeof( char ));

	// Create byte send buffer
	char queue_data_buffer;
	char uart_send_buffer[255];
	uint32_t bytes_remaining;

	// Enter while loop and send when new data is added to queue
	while(true)
	{
		vTaskDelay(10/portTICK_RATE_MS);
		uint8_t buffer_counter = 0;
		if( xQueueReceive( xQueue_uart_gsm_send_handle, &queue_data_buffer, LONG_TIME) )
		{
			*uart_send_buffer = queue_data_buffer;
			while(xQueueReceive( xQueue_uart_gsm_send_handle, &queue_data_buffer, 0))
			{
				buffer_counter++;
				*(uart_send_buffer + buffer_counter) = queue_data_buffer;
			}
			*(uart_send_buffer + buffer_counter+1) = NULL;

			// Send package to gsm module
			LPUART_DRV_SendData(FSL_LPUARTCOM1, &uart_send_buffer, (buffer_counter+1));

			// Add to virtual com queue
//			add_string_to_queue(uart_send_buffer, xQueue_virtual_com_send_handle, 0, xSemaphore_virtual_com_send_handle, 10);

			while((LPUART_DRV_GetTransmitStatus(FSL_LPUARTCOM1, &bytes_remaining) == kStatus_UART_TxBusy))
			{
				vTaskDelay(2/portTICK_RATE_MS);
			}
		}
	}
}
