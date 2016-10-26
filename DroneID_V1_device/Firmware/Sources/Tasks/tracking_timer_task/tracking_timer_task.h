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
* File: tracking_timer_task.h
* Purpose: Pass binary semaphore to send new pkg
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-11-26 Martin Skriver, Source written
****************************************************************************/

#ifndef TRACKING_TIMER_TASK_H_
#define TRACKING_TIMER_TASK_H_

/***************************************************************************/
/* External variables */
// If drone has moved then reset periodic send timer
extern xSemaphoreHandle xSemaphore_send_timer_reset;

/***************************************************************************/
/* Shared variables */
// Task handle
TaskHandle_t tracking_timer_handle;

// Create binary semaphore to send new UDP
xSemaphoreHandle xSemaphore_send_udp;

/***************************************************************************/
/* shared functions */
void tracking_timer_task(void *arg);

#endif /* TRACKING_TIMER_TASK_H_ */
