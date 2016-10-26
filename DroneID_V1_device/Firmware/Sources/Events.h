/****************************************************************************
* Callback functions
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
* File: Events.h
* Purpose: Handle data in callback functions
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-09-17 Martin Skriver, 	Source written
****************************************************************************/

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

/***************************************************************************/
/* application includes */
#include "fsl_device_registers.h"
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
#include "Tasks/GSM_uart_task/gsm_uart_receive.h"

#ifdef __cplusplus
extern "C" {
#endif 

/***************************************************************************/
/* Include external variables */
extern QueueHandle_t xQueue_uart_gsm_receive_handle;

/***************************************************************************/
/* Shared variables */
uint8_t rx_buffer[50];

/***************************************************************************/
void uartCom1_RxCallback(uint32_t instance, void * uartState);

/***************************************************************************/
/*! tpmTmr1 IRQ handler */
void TPM1_IRQHandler(void);

/***************************************************************************/
void LLWU_IRQHandler(void);

/***************************************************************************/
void lpuartCom1_RxCallback(uint32_t instance, void * lpuartState);

/***************************************************************************/
/*! adConv1 IRQ handler */
void ADC0_IRQHandler(void);

/***************************************************************************/
/*! i2cCom1 IRQ handler */
void I2C0_IRQHandler(void);

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
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
