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
* File: button_task.h
* Purpose: Detect button press and define type of press
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-11-09 Martin Skriver, Source written
****************************************************************************/

#ifndef BUTTON_TASK_H_
#define BUTTON_TASK_H_

/***************************************************************************/
/* Global types */
// Button push type
typedef enum _button_push_type
{
	NONE,
	SINGLE,
	DOUBLE,
	LONG,
	VERY_LONG
}button_push_type;

/***************************************************************************/
/* Global defined variables */

// Task handle to change task state from other tasks
TaskHandle_t button_main_task_handle;

// Binary semaphores to pass press type to other tasks
xSemaphoreHandle xSemaphore_single_press;
xSemaphoreHandle xSemaphore_double_press;
xSemaphoreHandle xSemaphore_long_press;
xSemaphoreHandle xSemaphore_very_long_press;

/***************************************************************************/
/* Shared functions */
void button_main_task(void *param);

#endif /* BUTTON_TASK_H_ */
