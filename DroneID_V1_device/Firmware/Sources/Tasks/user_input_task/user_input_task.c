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
* File: user_input_task.c
* Purpose: Pass input from buttons and sensors to manipulate DroneID states
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-10-15 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* system includes */

/***************************************************************************/
/* application includes */
#include "free_rtos.h"
#include "Drivers/SIM808/SIM808_driver.h"
#include "user_input_task.h"

/***************************************************************************/
/* #defines */
#define SEMAPHORE_BLOCK_TIME_0				 	0
#define SEMAPHORE_BLOCK_MS_LED_INDICATOR 		5
#define MS_LOOP_DELAY							50

/***************************************************************************/
/* function prototypes */
void create_binary_semaphores(void);
void wait_for_created_dependencies(void);
void enter_read_user_input(void);
void enter_sleep_mode(void);

/***************************************************************************/
/* Create binary semaphore to pass signals to other tasks */
void create_binary_semaphores(void)
{
	// Create semaphores to tap through DroneID states
	vSemaphoreCreateBinary(xSemaphore_next_state);
	vSemaphoreCreateBinary(xSemaphore_pre_state);

	// Mutex to protect variable for DroneID mode
	xSemaphore_droneid_mode_selector = xSemaphoreCreateMutex();
}

/***************************************************************************/
/* Wait until dependent instances from other tasks are created */
void wait_for_created_dependencies(void)
{
	while(!((xSemaphore_single_press != NULL) && (xSemaphore_double_press != NULL)
			&& (xSemaphore_long_press != NULL)))
	{
		vTaskDelay(1000/portTICK_RATE_MS);
	}
}

/***************************************************************************/
/* Suspend tasks and set mcu to sleep mode */
void enter_sleep_mode(void)
{
	// Go to sleep mode
	// Kill task does not work because of memory allocation is not allowed so suspend tasks is used.
	// Stop task to prevent power ports to me manipulated before entering sleep mode.
	vTaskSuspend(drone_id_task_handle);
	vTaskSuspend(GPS_task_handle);
	vTaskSuspend(vc_task_handle);
	vTaskSuspend(gsm_uart_send_task_handle);
	vTaskSuspend(gsm_uart_receive_task_handle);
	vTaskSuspend(button_main_task_handle);
	vTaskSuspend(adc_main_task_handle);
	vTaskSuspend(imu_main_task_handle);
	vTaskSuspend(tracking_timer_handle);

	// Power off gsm modem
	turn_off_modem_pwr_physical();

	// Power of led
	if(xSemaphoreTake(xSemaphore_lp_state_indicator, SEMAPHORE_BLOCK_MS_LED_INDICATOR))
	{
		lp_state_red_indicator = MODULE_IN_SLEEP_MODE;
		xSemaphoreGive( xSemaphore_lp_state_indicator );
	}
	vTaskDelay(500/portTICK_RATE_MS);
	POWER_SYS_SetMode(1, kPowerManagerPolicyAgreement);
}
/***************************************************************************/
/* Enter read input function and stay forever */
void enter_read_user_input(void)
{
	xSemaphoreTake(xSemaphore_next_state, SEMAPHORE_BLOCK_TIME_0);
	xSemaphoreTake(xSemaphore_pre_state, SEMAPHORE_BLOCK_TIME_0);
	xSemaphoreTake(xSemaphore_long_press, SEMAPHORE_BLOCK_TIME_0);

	while(true)
	{
		if(xSemaphoreTake(xSemaphore_single_press, SEMAPHORE_BLOCK_TIME_0))
		{
			// Stop DroneID
			if(xSemaphoreTake(xSemaphore_droneid_mode_selector, 1000))
			{
				droneid_mode_selector = POWER_ON;
				xSemaphoreGive(xSemaphore_droneid_mode_selector);
			}

			xSemaphoreTake(xSemaphore_pre_state, SEMAPHORE_BLOCK_TIME_0);
			xSemaphoreGive(xSemaphore_next_state);
		}
		else if(xSemaphoreTake(xSemaphore_double_press, SEMAPHORE_BLOCK_TIME_0))
		{
			// Stop DroneID
			if(xSemaphoreTake(xSemaphore_droneid_mode_selector, 1000))
			{
				droneid_mode_selector = POWER_OFF;
				xSemaphoreGive(xSemaphore_droneid_mode_selector);
			}

			// Give semaphore and just in case take back old semaphores
			xSemaphoreTake(xSemaphore_next_state, SEMAPHORE_BLOCK_TIME_0);
			xSemaphoreGive(xSemaphore_pre_state);
		}
		else if(xSemaphoreTake(xSemaphore_long_press, SEMAPHORE_BLOCK_TIME_0))
		{
			// Stop DroneID
			if(xSemaphoreTake(xSemaphore_droneid_mode_selector, 1000))
			{
				droneid_mode_selector = POWER_OFF;
				xSemaphoreGive(xSemaphore_droneid_mode_selector);
			}

			// Try send stop msg before shut dowm
			xSemaphoreTake(xSemaphore_next_state, SEMAPHORE_BLOCK_TIME_0);
			xSemaphoreGive(xSemaphore_pre_state);

			vTaskDelay(1000/portTICK_RATE_MS);

			enter_sleep_mode();
		}
		vTaskDelay(MS_LOOP_DELAY/portTICK_RATE_MS);
	}
}

/***************************************************************************/
/* Read user inputs and pass binary semaphores for DroniID states and handle sleep mode */
void user_input_main_task(void *arg)
{
	create_binary_semaphores();

	if(xSemaphoreTake(xSemaphore_droneid_mode_selector, 1000))
	{
		droneid_mode_selector = POWER_OFF;
		xSemaphoreGive(xSemaphore_droneid_mode_selector);
	}

	wait_for_created_dependencies();
	vTaskDelay(1000/portTICK_RATE_MS);
	enter_read_user_input();
}
/***************************************************************************/

