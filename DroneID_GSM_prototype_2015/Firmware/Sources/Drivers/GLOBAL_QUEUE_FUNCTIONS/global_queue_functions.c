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
* File: global_queue_functions.c
* Purpose: General queue functions to send, receive and find strings
* Project: DroneID
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2015-10-29 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* application includes */
#include "free_rtos.h"
#include "global_queue_functions.h"

/***************************************************************************/
// Function add string to queue TODO Change argument to ** array of pointers to call function with splitted data
bool add_string_to_queue(char *data, QueueHandle_t * queue, uint32_t queue_block_time, SemaphoreHandle_t * semaphore, uint8_t semaphore_block_time)
{
	bool return_value = false;
	uint32_t break_out_counter = 0;

	// Check if queue and mutex is created and if data is empty
	if( (queue != NULL) && (semaphore != NULL) && (*data != NULL))
	{
		// Take mutex before sending
		if(xSemaphoreTake(semaphore, ( TickType_t ) semaphore_block_time)  == pdTRUE)
		{
			// Add data to send queue byte by byte
			while((*data != NULL) && (break_out_counter < 200))
			{
				if(xQueueSend(queue, (void *) data, ( TickType_t ) queue_block_time) == pdPASS)
				{
					// Data added to queue successfully
					return_value = true;
					*data++;
				}
				break_out_counter++;
			}
			// Give back mutex when done sending
			xSemaphoreGive(semaphore);
		}
	}
	return return_value;
}

/***************************************************************************/
// New send function to send array of strings
bool add_array_of_strings_to_queue(char **data, QueueHandle_t * queue,uint32_t queue_block_time, SemaphoreHandle_t * semaphore, uint8_t semaphore_block_time)
{
	// Return value
	bool return_value = false;
	uint8_t string_number = 0;
	uint8_t data_counter = 0;
	uint32_t break_out_counter = 0;

	// Check if queue and mutex is created and if data is empty
	if( (queue != NULL) && (semaphore != NULL) && (NULL != *(data[string_number] + data_counter)))
	{
		// Take mutex before sending
		if(xSemaphoreTake(semaphore, ( TickType_t ) semaphore_block_time)  == pdTRUE)
		{
			// Add data to send queue byte by byte
			while((NULL != *(data[string_number] + data_counter)) && (break_out_counter < 500))
			{
				if(xQueueSend(queue, (void *) (data[string_number] + data_counter), ( TickType_t ) queue_block_time) == pdPASS)
				{
					data_counter++;
					if(*(data[string_number] + data_counter) == NULL)
					{
						string_number++;
						data_counter = 0;
						vTaskDelay(3/portTICK_RATE_MS);
						if(*data[string_number] == NULL)
						{
							return_value = true;
						}
					}
				}
				else
				{
					vTaskDelay(50/portTICK_RATE_MS);
				}
			}
			// TODO DELETE
			// Give back mutex when done sending
			xSemaphoreGive(semaphore);
		}
		else
		{
			vTaskDelay(50/portTICK_RATE_MS);
		}
	}

	return return_value;
}

/***************************************************************************/
// Function to add char to queue
bool add_to_queue(char data, QueueHandle_t * queue, uint8_t queue_block_time, SemaphoreHandle_t * semaphore, uint8_t semaphore_block_time)
{
	bool return_value = false;

	if( (queue != NULL) && (semaphore != NULL) )
	{
		if(xSemaphoreTake(semaphore, ( TickType_t ) semaphore_block_time)  == pdTRUE)
		{
			if(xQueueSend(queue, (void *) &data, ( TickType_t ) queue_block_time) == pdPASS)
			{
				// Failed to send to queue.
				return_value = true;
			}
			xSemaphoreGive(semaphore);
		}
	}
	return return_value;
}

/***************************************************************************/
// Function to reset queue and empty the buffer
bool clear_queue(QueueHandle_t * queue, SemaphoreHandle_t * semaphore, uint8_t semaphore_block_time)
{
	bool return_value = false;

	if( (queue != NULL) && (semaphore != NULL) )
	{
		if(xSemaphoreTake(semaphore, ( TickType_t ) semaphore_block_time)  == pdTRUE)
		{
			if(xQueueReset(queue) == pdPASS)
			{
				// Failed to send to queue.
				return_value = true;
			}
			xSemaphoreGive(semaphore);
		}
	}
	return return_value;
}

/***************************************************************************/
// Function to compare string with data from queue
bool queue_find_compare_match(char **data, QueueHandle_t * queue, uint32_t ms_time_out)
{
	bool return_value = false;
	char receive_buffer;
	uint32_t data_counter = 0;
	uint32_t timer_buffer = 0;
	uint8_t string_number = 0;

	// Dismiss if queue has not created
	if(queue != NULL)
	{
		// Break out if timer expires or match has been found in queue
		while((!return_value) && (*(data[string_number] + data_counter) != NULL) && (timer_buffer < ms_time_out))
		{
			// Check all items in queue until match is found or queue is empty
			while( xQueueReceive( queue, &receive_buffer, ( TickType_t ) 0 ) )
			{
				if(receive_buffer == *(data[string_number] + data_counter))
				{
					data_counter++;
					if(*(data[string_number] + data_counter) == NULL)
					{
						string_number++;
						data_counter = 0;
						if(*data[string_number] == NULL)
						{
							return_value = true;
						}
					}
				}
				else if(receive_buffer == *data[string_number])
				{
					data_counter = 1;
				}
				else
				{
					data_counter = 0;
				}
			}
			if(!return_value)
			{
				vTaskDelay(1/portTICK_RATE_MS);
				timer_buffer++;
			}
		}
	}
	return return_value;
}

/***************************************************************************/
// Returning string that starts and ends with predefines characters
bool return_string_with_identifier(QueueHandle_t * queue, char *return_buffer, char *start, char *end, uint8_t max_lenght, uint32_t max_time)
{
	bool return_value = false;
	char receive_buffer;
	bool start_accepted = false;
	uint32_t data_counter = 0;
	uint32_t end_data_counter = 0;
	uint32_t timer_buffer = 0;

	// Dismiss if queue has not created
	if(queue != NULL)
	{
		// Break out if timer expires or match has been found in queue
		while((!return_value) && (timer_buffer < max_time))
		{
			// Check all items in queue until match is found or queue is empty
			while( xQueueReceive( queue, &receive_buffer, ( TickType_t ) 0 ) )
			{
				// First check the string begin data
				if(!start_accepted)
				{
					// Check if the receive data match the start data
					if(receive_buffer == *(start + data_counter))
					{
						*(return_buffer + data_counter) = receive_buffer;
						data_counter++;
						// Check if all data from beginning of the string is matching
						if(*(start + data_counter) == NULL)
						{
							start_accepted = true;
							data_counter--;
						}
					}
					// If data has startet accepting a wrong string and the right one starts during this process
					else if(receive_buffer == *start)
					{
						data_counter = 1;
						*return_buffer = receive_buffer;
					}
					// This data was wrong start for the beginning of the string
					else
					{
						data_counter = 0;
					}
				}
				// First part of string was accepted start with part between and end part
				else
				{
					data_counter++;
					*(return_buffer + data_counter) = receive_buffer;
					// Check if the receive data match the end data
					if(receive_buffer == *(end + end_data_counter))
					{
						end_data_counter++;
						// Check if all end chars is accepted, set end of string character and return accepted
						if(*(end + end_data_counter) == NULL)
						{
							data_counter++;
							*(return_buffer + data_counter) = NULL;
							return_value = true;
						}
					}
					// If data has startet accepting a wrong string and the right one starts during this process
					else if(receive_buffer == *end)
					{
						end_data_counter = 1;
					}
					else
					{
						end_data_counter = 0;
					}
				}
				// If size is bigger than max reset and start from beginning
				if(max_lenght < data_counter)
				{
					data_counter = 0;
					start_accepted = false;
				}
			}
			if(!return_value)
			{
				vTaskDelay(1/portTICK_RATE_MS);
				timer_buffer++;
			}
		}
	}
	return return_value;
}

