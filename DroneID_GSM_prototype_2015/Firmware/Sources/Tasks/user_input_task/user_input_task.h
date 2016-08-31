/****************************************************************************
* DroneID handler for user inputs
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
* File: user_input_task.h
* Purpose: Pass input from buttons and sensors to manipulate DroneID states
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-10-15 Martin Skriver, Source written
****************************************************************************/

#ifndef USER_INPUT_TASK_H_
#define USER_INPUT_TASK_H_

/***************************************************************************/
/* application includes */
#include "Tasks/button_task/button_task.h"
#include "Tasks/LED_task/led_task.h"

/***************************************************************************/
/* Global types */
// Button push type
typedef enum _droneid_mode_selector_type
{
	POWER_OFF,
	POWER_ON
}droneid_mode_selector_type;

/***************************************************************************/
/* Include external variables */
// Handles to kill license plate tasks before go to sleep
extern TaskHandle_t drone_id_task_handle;
extern TaskHandle_t GPS_task_handle;
extern TaskHandle_t vc_task_handle;
extern TaskHandle_t gsm_uart_send_task_handle;
extern TaskHandle_t gsm_uart_receive_task_handle;
extern TaskHandle_t button_main_task_handle;
extern TaskHandle_t adc_main_task_handle;
extern TaskHandle_t imu_main_task_handle;
extern TaskHandle_t tracking_timer_handle;

// Binary semaphore to start and stop the license plate
extern xSemaphoreHandle xSemaphore_single_press;
extern xSemaphoreHandle xSemaphore_double_press;
extern xSemaphoreHandle xSemaphore_long_press;

// Protected variable for the red led indicator
extern lp_state_indicator_types lp_state_red_indicator;
extern SemaphoreHandle_t xSemaphore_lp_state_indicator;

/***************************************************************************/
/* Shared variables */
// Create binary semaphores to tap through DroneID states
xSemaphoreHandle xSemaphore_pre_state;
xSemaphoreHandle xSemaphore_next_state;

// Protected variable for user selected DroneID mode
droneid_mode_selector_type droneid_mode_selector;
SemaphoreHandle_t xSemaphore_droneid_mode_selector;

/***************************************************************************/
/* Shared functions */
void user_input_main_task(void *arg);

#endif /* USER_INPUT_TASK_H_ */
