/****************************************************************************
* Main file for DroneID
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
* File: main.c
* Purpose: Init hardware, OS and task create
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-16 Martin Skriver, 	Source written
****************************************************************************/

/***************************************************************************/
/* application includes */
/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "osa1.h"
#include "free_rtos.h"
#include "DbgCs1.h"
#include "gpio1.h"
#include "tpmTmr1.h"
#include "pwrMan1.h"
#include "lpuartCom1.h"
#include "adConv1.h"
#include "i2cCom1.h"
#include "Init_Config.h"
/* User includes (#include below this line is not maintained by Processor Expert) */
#include "Tasks/drone_id_task/drone_id_task.h"
#include "Tasks/Virtual_Com/virtual_com.h"
#include "Tasks/GPS_task/gps.h"
#include "Tasks/GSM_uart_task/gsm_uart_send.h"
#include "Tasks/GSM_uart_task/gsm_uart_receive.h"
#include "Tasks/user_input_task/user_input_task.h"
#include "Tasks/LED_task/led_task.h"
#include "Tasks/button_task/button_task.h"
#include "Tasks/ADC_task/adc_task.h"
#include "Tasks/IMU_task/imu_task.h"
#include "Tasks/tracking_timer_task/tracking_timer_task.h"

/***************************************************************************/
/* Main function */
/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
	xTaskCreate(
		gps_task,  /* task function */
		"GPS_TASK", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		&GPS_task_handle); /* task handle */
	configASSERT( GPS_task_handle );

	xTaskCreate(
		drone_id_main_task,  /* task function */
		"DRONE_ID_TASK", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		&drone_id_task_handle); /* task handle */
	configASSERT( drone_id_task_handle );

	xTaskCreate(
		user_input_main_task,  /* task function */
		"USER_INPUT_TASK", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		NULL); /* task handle */

	xTaskCreate(
		vc_task,  /* task function */
		"VC_TASK", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		&vc_task_handle); /* task handle */
	configASSERT( vc_task_handle );

	xTaskCreate(
		gsm_uart_send_task,  /* task function */
		"GSM_UART_SEND", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		&gsm_uart_send_task_handle); /* task handle */
	configASSERT( gsm_uart_send_task_handle );

	xTaskCreate(
		gsm_uart_receive_task,  /* task function */
		"GSM_UART_RECEIVE", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		&gsm_uart_receive_task_handle); /* task handle */
	configASSERT( gsm_uart_receive_task_handle );

	xTaskCreate(
		led_main_task, /* task function */
		"LED_TASK", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		NULL); /* task handle */

	xTaskCreate(
		button_main_task,  /* task function */
		"BUTTON_TASK", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		&button_main_task_handle); /* task handle */
	configASSERT( button_main_task_handle );

	xTaskCreate(
		adc_main_task,  /* task function */
		"ADC_TASK", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		configMAX_PRIORITIES - 10, //tskIDLE_PRIORITY,  /* initial priority */
		&adc_main_task_handle); /* task handle */
	configASSERT( adc_main_task_handle );

	xTaskCreate(
		tracking_timer_task,  /* task function */
		"TRACKING_TIMER", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		&tracking_timer_handle); /* task handle */
	configASSERT( tracking_timer_handle );

	xTaskCreate(
		imu_main_task,  /* task function */
		"IMU_TASK", /* task name for kernel awareness */
		configMINIMAL_STACK_SIZE, /* task stack size */
		(void*)NULL, /* optional task startup argument */
		tskIDLE_PRIORITY+10,  /* initial priority */
		&imu_main_task_handle); /* task handle */
	configASSERT( imu_main_task_handle );


  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/
/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
