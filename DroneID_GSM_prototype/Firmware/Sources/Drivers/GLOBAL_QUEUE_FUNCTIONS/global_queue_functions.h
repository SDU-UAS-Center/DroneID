/****************************************************************************
* Queue functions
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
* File: global_queue_functions.h
* Purpose: General queue functions to send, receive and find strings
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-10-29 Martin Skriver, Source written
****************************************************************************/

#ifndef GLOBAL_QUEUE_FUNCTIONS_H_
#define GLOBAL_QUEUE_FUNCTIONS_H_

/***************************************************************************/
// Function add string to queue
bool add_string_to_queue(char *data, QueueHandle_t * queue, uint32_t queue_block_time, SemaphoreHandle_t * semaphore, uint8_t semaphore_block_time);

/***************************************************************************/
// New send function to send array of strings
bool add_array_of_strings_to_queue(char **data, QueueHandle_t * queue,uint32_t queue_block_time, SemaphoreHandle_t * semaphore, uint8_t semaphore_block_time);

/***************************************************************************/
// Function to add char to queue
bool add_to_queue(char data, QueueHandle_t * queue, uint8_t queue_block_time, SemaphoreHandle_t * semaphore, uint8_t semaphore_block_time);

/***************************************************************************/
// Function to reset queue and empty the buffer
bool clear_queue(QueueHandle_t * queue, SemaphoreHandle_t * semaphore, uint8_t semaphore_block_time);

/***************************************************************************/
// Function to compare strings with data from queue
bool queue_find_compare_match(char **data, QueueHandle_t * queue, uint32_t ms_time_out);

/***************************************************************************/
// Returning string that starts and ends with predefines characters
bool return_string_with_identifier(QueueHandle_t * queue, char *return_buffer, char *start, char *end, uint8_t max_lenght, uint32_t max_time);

#endif /* GLOBAL_QUEUE_FUNCTIONS_H_ */
