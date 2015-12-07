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
#define GREEN_LED_CHARGING_DUTY		700
#define GREEN_LED_BATTERY_DUTY		100
#define RED_LED_CHARGING_DUTY		700
#define RED_LED_BATTERY_DUTY		100

#define MS_DELAY					10
#define MS_PULSE_LED_ON				200
#define MS_PULSE_LED_OFF			200
#define MS_PULSE_PAUSE				1000

/***************************************************************************/
// Make pulse/pause with argument of no. of pulses
bool pulse_mode_get_led_state(uint8_t no_of_pulses)
{
	static uint8_t pulse_no = 0;
	static bool led_on = false;
	static uint32_t ms_count = 0;
	//

	// Check if pause time or pulse time
	if(no_of_pulses >= pulse_no)
	{
		// Check if pulse high or low
		if(led_on)
		{
			if(ms_count >= MS_PULSE_LED_ON)
			{
				ms_count = 0;
				led_on = false;
				pulse_no++;
			}
		}
		else
		{
			if(ms_count >= MS_PULSE_LED_OFF)
			{
				ms_count = 0;
				led_on = true;
			}
		}
	}
	// Pause time
	else
	{
		if( ms_count >= MS_PULSE_PAUSE)
		{
			ms_count = 0;
			pulse_no = 1;
			led_on = true;
		}
	}

	ms_count += MS_DELAY;
	return led_on;
}

/***************************************************************************/
// Gives a sinus function to the intensity of light
uint32_t module_powered_on_sin(uint32_t charging_state, uint8_t speed)
{
	static uint32_t duty_buffer = 0;
	if(duty_buffer >= 359)
		{
			duty_buffer = 0;
		}
	else
		{
			duty_buffer += speed;
		}
	if(charging_state)
	{
		return (RED_LED_BATTERY_DUTY/2)*(1+sin(duty_buffer/57.295f));
	}
	else
	{
		return (RED_LED_CHARGING_DUTY/2)*(1+sin(duty_buffer/57.295f));
	}
}

/***************************************************************************/
// Indicate with the led in which state the DroneID is in
uint16_t get_red_signal(uint32_t charging_state)
{
	bool led_indicator = false;

	switch (lp_state_red_indicator)
	{
		// Set mcu io to power on gsm modem
	case MODULE_IN_SLEEP_MODE:
		led_indicator = 0;
		break;

	case MODULE_POWERED_OFF:
		led_indicator = pulse_mode_get_led_state(1);
		break;

	case MODULE_ENTERING_UPDATE_MODE:
		led_indicator = pulse_mode_get_led_state(2);
		break;

	case MODULE_ENTERING_POWER_OFF_MODE:
		led_indicator = pulse_mode_get_led_state(3);
		break;

	case UPDATING_WITH_FIX:
		return module_powered_on_sin(charging_state, 5);
		break;

	case UPDATING_WITHOUT_FIX:
		return module_powered_on_sin(charging_state, 1);
		break;
	}

	if(charging_state)
	{
		return led_indicator*RED_LED_BATTERY_DUTY;
	}
	else
	{
		return led_indicator*RED_LED_CHARGING_DUTY;
	}
	return pulse_mode_get_led_state(5)*RED_LED_CHARGING_DUTY;
}

/***************************************************************************/
// Connect green led to gsm netlight pin
uint16_t get_green_signal(uint32_t charging_state)
{
	if(charging_state)
	{
		return GPIO_DRV_ReadPinInput(SIM808_NETLIGHT)*GREEN_LED_BATTERY_DUTY;
	}
	else
	{
		return GPIO_DRV_ReadPinInput(SIM808_NETLIGHT)*GREEN_LED_CHARGING_DUTY;
	}
}

/***************************************************************************/
void led_main_task(void *param)
{
	xSemaphore_lp_state_indicator = xSemaphoreCreateMutex();
	if(xSemaphoreTake(xSemaphore_lp_state_indicator, 0))
	{
		lp_state_red_indicator = MODULE_POWERED_OFF;
		xSemaphoreGive( xSemaphore_lp_state_indicator );
	}

	while(true)
	{
		TPM1_C0V = get_red_signal(GPIO_DRV_ReadPinInput(LTC4065_CHRG));
		TPM1_C1V = get_green_signal(GPIO_DRV_ReadPinInput(LTC4065_CHRG));

		vTaskDelay(MS_DELAY/portTICK_RATE_MS); /* wait for 250 ms */
	}
}

