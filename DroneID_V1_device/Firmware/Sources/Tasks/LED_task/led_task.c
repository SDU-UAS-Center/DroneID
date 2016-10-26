/****************************************************************************
* DroneID LED indicator
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
* File: led_task.c
* Purpose: Indicate the state of the DroneID
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-11 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* system includes */
#include "clockMan1.h"
#include "pwrMan1.h"
#include "math.h"
#include "gpio1.h"
#include "free_rtos.h"
#include "led_task.h"

/***************************************************************************/
/* #defines */
// Define pwm duty for leds
#define LED_DUTY					100
#define LED_OFF_duty				0

#define DELAY_10_MS					10
#define DELAY_300_MS				300

#define PULSE_PAUSE_LED_ON_MS		200
#define PULSE_PAUSE_LED_OFF_MS		200
#define PULSE_PAUSE_DUTY_MS			1000
#define ONE_PULSE					1
#define TWO_PULSES					2
#define THREE_PULSES				3


#define FLASH_MODE_ON_MS			10
#define FLASH_MODE_OFF_MS			90

/***************************************************************************/
// Flashing return value according on and off time
bool flash_mode(uint32_t on_ms, uint32_t off_ms)
{
	static bool led_on = false;
	static uint32_t ms_count = 0;

	if(led_on)
	{
		if( ms_count >= on_ms)
		{
			ms_count = 0;
			led_on = false;
		}
	}
	else
	{
		if( ms_count >= off_ms)
		{
			ms_count = 0;
			led_on = true;
		}
	}
	ms_count += DELAY_10_MS;
	return led_on;
}

/***************************************************************************/
// Make pulse/pause with argument of no. of pulses
bool pulse_puse_mode(uint8_t no_of_pulses)
{
	static uint8_t pulse_no = 0;
	static bool led_on = false;
	static uint32_t ms_count = 0;

	// Check if pause time or pulse time
	if(no_of_pulses >= pulse_no)
	{
		// Check if pulse high or low
		if(led_on)
		{
			if(ms_count >= PULSE_PAUSE_LED_ON_MS)
			{
				ms_count = 0;
				led_on = false;
				pulse_no++;
			}
		}
		else
		{
			if(ms_count >= PULSE_PAUSE_LED_OFF_MS)
			{
				ms_count = 0;
				led_on = true;
			}
		}
	}
	// Pause time
	else
	{
		if( ms_count >= PULSE_PAUSE_DUTY_MS)
		{
			ms_count = 0;
			pulse_no = 1;
			led_on = true;
		}
	}

	ms_count += DELAY_10_MS;
	return led_on;
}

/***************************************************************************/
// Read status of DroneID and set LED state indicator
void update_DroneID_led()
{
	// Get charging indication
	// GPIO_DRV_ReadPinInput(LTC4065_CHRG)
	if(xSemaphoreTake(xSemaphore_lp_state_indicator, 0))
	{
		switch (lp_state_red_indicator)
		{
		case MODULE_IN_SLEEP_MODE:
			TPM1_C0V = LED_OFF_duty;
			TPM1_C1V = LED_OFF_duty;
			break;

		case MODULE_POWERED_OFF:
			TPM1_C0V = LED_DUTY * pulse_puse_mode(ONE_PULSE);
			TPM1_C1V = LED_OFF_duty;
			break;

		case MODULE_CONNECTING_GPRS:
			TPM1_C0V = LED_DUTY * pulse_puse_mode(TWO_PULSES);
			TPM1_C1V = LED_OFF_duty;
			break;

		case MODULE_CONNECTING_GPS:
			TPM1_C0V = LED_DUTY * pulse_puse_mode(THREE_PULSES);
			TPM1_C1V = LED_OFF_duty;
			break;

		case TRACKING_NO_FLIGHT_ZONE:
			TPM1_C0V = LED_DUTY * flash_mode(FLASH_MODE_ON_MS, FLASH_MODE_OFF_MS);
			TPM1_C1V = LED_OFF_duty;
			break;

		case TRACKING_FLIGHT_ZONE:
			TPM1_C0V = LED_OFF_duty;
			TPM1_C1V = LED_DUTY * flash_mode(FLASH_MODE_ON_MS, FLASH_MODE_OFF_MS);
			break;
		}
		xSemaphoreGive( xSemaphore_lp_state_indicator);
	}
}

/***************************************************************************/
// Main task for controlling the two LEDs on the DroneID
void led_main_task(void *param)
{
	// Create mutex and instantiate LED status
	xSemaphore_lp_state_indicator = xSemaphoreCreateMutex();
	vTaskDelay(DELAY_300_MS/portTICK_RATE_MS);
	if(xSemaphoreTake(xSemaphore_lp_state_indicator, 0))
	{
		lp_state_red_indicator = MODULE_POWERED_OFF;
		xSemaphoreGive( xSemaphore_lp_state_indicator);
	}

	// Enter while loop forever
	while(true)
	{
		update_DroneID_led();
		vTaskDelay(DELAY_10_MS/portTICK_RATE_MS);
	}
}

