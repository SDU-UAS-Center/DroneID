/****************************************************************************
* DroneID detector for button press
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
* File: button_task.c
* Purpose: Detect button press and define type of press
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-11-09 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* system includes */

/***************************************************************************/
/* application includes */
#include "gpio1.h"
#include "free_rtos.h"
#include "button_task.h"

/***************************************************************************/
/* #defines */
#define LONG_PRESS_MS				1500
#define VERY_LONG_PRESS_MS			4500
#define MAX_DELAY_BETWEEN_PUSH		500
#define JITTER_FILTER_MS			30
#define MS_LOOP_DELAY				10
#define MS_MIN_TO_NX_PUSH_SEQ		500

/***************************************************************************/
/* #types */
typedef enum _button_push_cycle
{
	SINGLE_STATE,
	SINGLE_STATE_WAIT,
	DOUBLE_STATE
}button_push_cycle;

/***************************************************************************/
/* function prototypes */
button_push_type get_push_type(uint32_t button);
void create_binary_button_task_semaphores(void);
void read_button(void);

/***************************************************************************/
/* Read button input and return press type */
button_push_type get_push_type(uint32_t button)
{
	button_push_cycle button_states = SINGLE_STATE;
	uint32_t button_timer = 0;
	uint32_t filter_buffer = 0;

	while(true)
	{
		switch(button_states)
		{
		case SINGLE_STATE:
			if(!GPIO_DRV_ReadPinInput(button))
			{
				if(filter_buffer >= JITTER_FILTER_MS)
				{
					filter_buffer = 0;
					button_states = SINGLE_STATE_WAIT;
				}
				filter_buffer += MS_LOOP_DELAY;
			}
			else
			{
				return NONE;
			}
			break;

		case SINGLE_STATE_WAIT:
			if(GPIO_DRV_ReadPinInput(button))
			{
				if(filter_buffer >= JITTER_FILTER_MS)
				{
					// If pressed more than defined for long press
					if(button_timer >= LONG_PRESS_MS)
					{
						return LONG;
					}

					button_timer = 0;
					filter_buffer = 0;
					button_states = DOUBLE_STATE;
				}
				filter_buffer += MS_LOOP_DELAY;
			}
			else
			{
				filter_buffer = 0;
				if(button_timer >= VERY_LONG_PRESS_MS )
				{
					return VERY_LONG;
				}
			}
			break;

		case DOUBLE_STATE:
			if(!GPIO_DRV_ReadPinInput(button))
			{
				if(filter_buffer >= JITTER_FILTER_MS)
				{
					return DOUBLE;
				}
				filter_buffer += MS_LOOP_DELAY;
			}
			if(button_timer >= MAX_DELAY_BETWEEN_PUSH)
			{
				return SINGLE;
			}
			break;
		}
		button_timer += MS_LOOP_DELAY;
		vTaskDelay(MS_LOOP_DELAY/portTICK_RATE_MS);
	}
	return NONE;
}

/***************************************************************************/
/* Create binary semaphore to pass signals to other tasks */
void create_binary_button_task_semaphores(void)
{
	// Create semaphores for each press type
	vSemaphoreCreateBinary(xSemaphore_single_press);
	vSemaphoreCreateBinary(xSemaphore_double_press);
	vSemaphoreCreateBinary(xSemaphore_long_press);
	vSemaphoreCreateBinary(xSemaphore_very_long_press);

	// Take semaphores just in case
	xSemaphoreTake(xSemaphore_single_press, 0);
	xSemaphoreTake(xSemaphore_double_press, 0);
	xSemaphoreTake(xSemaphore_long_press, 0);
	xSemaphoreTake(xSemaphore_very_long_press, 0);
}

/***************************************************************************/
/* Read if button is pressed and pass binary semaphore for press type */
void read_button(void)
{
	button_push_type push_button = NONE;
	while(true)
	{
		if(!GPIO_DRV_ReadPinInput(PUSH_BTN))
		{
			push_button = get_push_type(PUSH_BTN);
			switch(push_button)
			{
			case NONE:
				break;
			case SINGLE:
				xSemaphoreGive(xSemaphore_single_press);
				vTaskDelay(MS_MIN_TO_NX_PUSH_SEQ/portTICK_RATE_MS);
				break;
			case DOUBLE:
				xSemaphoreGive(xSemaphore_double_press);
				vTaskDelay(MS_MIN_TO_NX_PUSH_SEQ/portTICK_RATE_MS);
				break;
			case LONG:
				xSemaphoreGive(xSemaphore_long_press);
				vTaskDelay(MS_MIN_TO_NX_PUSH_SEQ/portTICK_RATE_MS);
				break;
			case VERY_LONG:
				xSemaphoreGive(xSemaphore_very_long_press);
				vTaskDelay(MS_MIN_TO_NX_PUSH_SEQ/portTICK_RATE_MS);
				break;
			}
			// Ensure button is released before exit
			while(!GPIO_DRV_ReadPinInput(PUSH_BTN))
			{
				vTaskDelay(50/portTICK_RATE_MS);
			}
			vTaskDelay(200/portTICK_RATE_MS);
		}
		vTaskDelay(20/portTICK_RATE_MS);
	}
}

/***************************************************************************/
/* Enter button task function*/
void button_main_task(void *param)
{
	create_binary_button_task_semaphores();
	read_button();
}












