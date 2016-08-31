/****************************************************************************
* DroneID main state machine
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
* File: drone_id_task.h
* Purpose: Controlling which commands to send to the GSM modem
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-10-17 Martin Skriver, Source written
****************************************************************************/

#ifndef DRONE_ID_TASK_H_
#define DRONE_ID_TASK_H_

/***************************************************************************/
/* application includes */
#include "Tasks/user_input_task/user_input_task.h"
#include "Tasks/GSM_uart_task/gsm_uart_send.h"
#include "Tasks/GSM_uart_task/gsm_uart_receive.h"
#include "Tasks/GPS_task/gps.h"
#include "semphr.h"
#include "Drivers/NMEA/nmea.h"
#include "Tasks/LED_task/led_task.h"


/***************************************************************************/
/* Include external variables */
// Binary semaphores from license plate task to start/stop transmitting
extern xSemaphoreHandle xSemaphore_pre_state;
extern xSemaphoreHandle xSemaphore_next_state;

// Protected queue to data send to GSM from gsm send
extern QueueHandle_t xQueue_uart_gsm_send_handle;
extern SemaphoreHandle_t xSemaphore_uart_gsm_send_handle;

// Protected variable for the red led indicator
extern lp_state_indicator_types lp_state_red_indicator;
extern SemaphoreHandle_t xSemaphore_lp_state_indicator;

extern SemaphoreHandle_t xSemaphore_batt_volt_mutex_handle;
extern double batt_voltage;

extern SemaphoreHandle_t xSemaphore_pressure_temperature_mutex_handle;
extern double temperature, pressure;

extern xSemaphoreHandle xSemaphore_send_udp;

// Binary semaphores to tap through DroneID states
extern xSemaphoreHandle xSemaphore_get_new_voltage_sample;

// GGA msg and mutex
extern SemaphoreHandle_t xSemaphore_gga_msg_mutex_handle;
extern gpgga_t gga_msg_global;

// DroneID has moved binary
extern xSemaphoreHandle xSemaphore_has_moved;

// Protected variable for user selected DroneID mode
extern droneid_mode_selector_type droneid_mode_selector;
extern SemaphoreHandle_t xSemaphore_droneid_mode_selector;

/***************************************************************************/
/* Shared variables */

// Handle for other tasks to kill/it
TaskHandle_t drone_id_task_handle;

// Binary semaphore to reset gpgga data buffer
xSemaphoreHandle xSemaphore_gpgga_reset;

// If drone has moved then reset periodic send timer
xSemaphoreHandle xSemaphore_send_timer_reset;

/***************************************************************************/
/* shared funtions */
void drone_id_main_task(void *arg);

#endif /* DRONE_ID_TASK_H_ */
