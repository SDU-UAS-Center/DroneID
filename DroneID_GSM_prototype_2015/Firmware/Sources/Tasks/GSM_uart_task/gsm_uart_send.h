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
* File: gsm_uart_send.h
* Purpose: Add bytes from send queue to UART send buffer
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-24 Martin Skriver, Source written
****************************************************************************/

#ifndef GSM_UART_SEND_H_
#define GSM_UART_SEND_H_

/***************************************************************************/
/* Include external variables */
// Protected queue to data send via virtual com
extern SemaphoreHandle_t xSemaphore_virtual_com_send_handle;
extern QueueHandle_t xQueue_virtual_com_send_handle;

/***************************************************************************/
/* Shared variables */
TaskHandle_t gsm_uart_send_task_handle;

// Protected queue to data send to GSM
QueueHandle_t xQueue_uart_gsm_send_handle;
SemaphoreHandle_t xSemaphore_uart_gsm_send_handle;

/***************************************************************************/
/* shared functions */
// Main task to send via uart
void gsm_uart_send_task(void *arg);

#endif /* GSM_UART_SEND_H_ */
