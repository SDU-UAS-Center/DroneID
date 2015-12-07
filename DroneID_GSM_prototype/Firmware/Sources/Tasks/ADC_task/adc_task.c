/****************************************************************************
* ADC task for reading battery voltage
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
* File: adc_task.c
* Purpose: Pass input from buttons and sensors to manipulate DroneID states
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-11-10 Martin Skriver, 	Source written
* Created:	2015-12-04 Martin Skriver, 	Changed from timer to binary semaphore
* 										dependent.
****************************************************************************/

/***************************************************************************/
/* application includes */
#include "gpio1.h"
#include "adConv1.h"
#include "adc_task.h"

void get_adc_mess(void)
{
    GPIO_DRV_SetPinOutput(BATT_VOLT_EN);
	ADC16_DRV_SetChnMux(FSL_ADCONV1,1);
	vTaskDelay(5/portTICK_RATE_MS);

	ADC16_DRV_ConfigConvChn(FSL_ADCONV1,0, &adConv1_ChnConfig0);
	ADC16_DRV_WaitConvDone(FSL_ADCONV1,0);

	if(xSemaphoreTake(xSemaphore_batt_volt_mutex_handle, 0))
	{
		batt_voltage = (2*ADC16_DRV_GetConvValueRAW(FSL_ADCONV1, 0)) * 0.000045777; // Remember magic number
		ADC16_DRV_PauseConv(FSL_ADCONV1,0);
		xSemaphoreGive( xSemaphore_batt_volt_mutex_handle );
	}
	GPIO_DRV_ClearPinOutput(BATT_VOLT_EN);
}

/***************************************************************************/
/* Function for reading adc for battery voltage */
void adc_main_task(void *param)
{
	// Create binary semaphore
	vSemaphoreCreateBinary(xSemaphore_get_new_voltage_sample);

	// Create batt voltage semaphore
	xSemaphore_batt_volt_mutex_handle = xSemaphoreCreateMutex();

	// Set voltage to 0
	if(xSemaphoreTake(xSemaphore_batt_volt_mutex_handle, 0))
	{
		batt_voltage = 0;
		xSemaphoreGive( xSemaphore_batt_volt_mutex_handle );
	}

	// Get first mess
	get_adc_mess();

	while(true)
	{
		if(xSemaphoreTake(xSemaphore_get_new_voltage_sample, 500))
		{
			get_adc_mess();
		}
	}
}
