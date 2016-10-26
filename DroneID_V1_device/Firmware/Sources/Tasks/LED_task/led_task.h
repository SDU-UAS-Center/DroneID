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
* File: led_task.h
* Purpose: Indicate the state of the DroneID
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-11 Martin Skriver, Source written
****************************************************************************/

#ifndef LED_TASK_h
#define LED_TASK_h

/***************************************************************************/
// Shared types
typedef enum _lp_state_indicator_types
{
	MODULE_IN_SLEEP_MODE,
	MODULE_POWERED_OFF,
	MODULE_CONNECTING_GPRS,
	MODULE_CONNECTING_GPS,
	TRACKING_NO_FLIGHT_ZONE,
	TRACKING_FLIGHT_ZONE
}lp_state_indicator_types;

/***************************************************************************/
/* Shared variables */
// Protected led indicator variable
lp_state_indicator_types lp_state_red_indicator;
SemaphoreHandle_t xSemaphore_lp_state_indicator;

/***************************************************************************/
/* shared functons */
// LED main task
void led_main_task(void *param);

#endif
