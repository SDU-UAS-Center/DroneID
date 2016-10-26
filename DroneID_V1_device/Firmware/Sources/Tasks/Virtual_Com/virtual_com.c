/**HEADER********************************************************************
 *
 * Copyright (c) 2008, 2013 - 2015 Freescale Semiconductor;
 * All Rights Reserved
 *
 * Copyright (c) 1989-2008 ARC International;
 * All Rights Reserved
 *
 ***************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************
 *
 * $FileName: virtual_com.c$
 * $Version :
 * $Date    :
 *
 * Comments:
 *
 * @brief  The file emulates a USB PORT as RS232 PORT.
 *****************************************************************************
 * Log:
 * Created:  2015-10-17 Martin Skriver, File created
 * Created:  2015-11-20 Martin Skriver, Modified to handle asynchronous send/receive
 ****************************************************************************/

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_stack_interface.h"
//#include "main.h"
#include "free_rtos.h"
#include "queue.h"
#include "Drivers/USB_CDC/usb_descriptor.h"
#include "virtual_com.h"


/****************************************************************************
 * Global Variables
 ****************************************************************************/
#define LONG_TIME 0xffff


extern usb_desc_request_notify_struct_t desc_callback;
extern uint8_t USB_Desc_Set_Speed(uint32_t handle, uint16_t speed);
cdc_handle_t g_app_handle;

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg);
uint8_t USB_App_Class_Callback(uint8_t event, uint16_t value, uint8_t ** data, uint32_t* size, void* arg);
void Virtual_Com_App(void);
/*****************************************************************************
 * Local Variables
 *****************************************************************************/
uint8_t g_line_coding[LINE_CODING_SIZE] =
{
    /*e.g. 0x00,0x10,0x0E,0x00 : 0x000E1000 is 921600 bits per second */
    (LINE_CODE_DTERATE_IFACE >> 0) & 0x000000FF,
    (LINE_CODE_DTERATE_IFACE >> 8) & 0x000000FF,
    (LINE_CODE_DTERATE_IFACE >> 16) & 0x000000FF,
    (LINE_CODE_DTERATE_IFACE >> 24) & 0x000000FF,
    LINE_CODE_CHARFORMAT_IFACE,
    LINE_CODE_PARITYTYPE_IFACE,
    LINE_CODE_DATABITS_IFACE
};

uint8_t g_abstract_state[COMM_FEATURE_DATA_SIZE] =
{
    (STATUS_ABSTRACT_STATE_IFACE >> 0) & 0x00FF,
    (STATUS_ABSTRACT_STATE_IFACE >> 8) & 0x00FF
};

uint8_t g_country_code[COMM_FEATURE_DATA_SIZE] =
{
    (COUNTRY_SETTING_IFACE >> 0) & 0x00FF,
    (COUNTRY_SETTING_IFACE >> 8) & 0x00FF
};
static bool start_app = FALSE;
static bool start_transactions = FALSE;

static uint8_t g_curr_recv_buf[DATA_BUFF_SIZE];
static uint8_t g_curr_send_buf[DATA_BUFF_SIZE];

static uint32_t g_recv_size;
static uint32_t g_send_size;

static uint16_t g_cdc_device_speed;
static uint16_t g_bulk_out_max_packet_size;
static uint16_t g_bulk_in_max_packet_size;

static bool received_data = TRUE; //ms implmented
int received = 0;
/*****************************************************************************
 * Local Functions
 *****************************************************************************/

/**************************************************************************//*!
 *
 * @name  USB_Get_Line_Coding
 *
 * @brief The function returns the Line Coding/Configuration
 *
 * @param handle:        handle
 * @param interface:     interface number
 * @param coding_data:   output line coding data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Line_Coding(uint32_t handle,
    uint8_t interface,
    uint8_t * *coding_data)
{
    //UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get line coding data*/
        *coding_data = g_line_coding;
        return USB_OK;
    }

    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Line_Coding
 *
 * @brief The function sets the Line Coding/Configuration
 *
 * @param handle: handle
 * @param interface:     interface number
 * @param coding_data:   output line coding data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Line_Coding(uint32_t handle,
    uint8_t interface,
    uint8_t * *coding_data)
{
    uint8_t count;

    //UNUSED_ARGUMENT(handle)

    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* set line coding data*/
        for (count = 0; count < LINE_CODING_SIZE; count++)
        {
            g_line_coding[count] = *((*coding_data + USB_SETUP_PKT_SIZE) + count);
        }
        return USB_OK;
    }

    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Get_Abstract_State
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (ABSTRACT_STATE)
 * @param handle:        handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Abstract_State(uint32_t handle,
    uint8_t interface,
    uint8_t * *feature_data)
{
    //UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get line coding data*/
        *feature_data = g_abstract_state;
        return USB_OK;
    }

    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Get_Country_Setting
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (COUNTRY_CODE)
 * @param handle:        handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Country_Setting(uint32_t handle,
    uint8_t interface,
    uint8_t * *feature_data)
{
    //UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get line coding data*/
        *feature_data = g_country_code;
        return USB_OK;
    }

    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Abstract_State
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (ABSTRACT_STATE)
 * @param handle:        handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Abstract_State(uint32_t handle,
    uint8_t interface,
    uint8_t * *feature_data)
{
    uint8_t count;
    //UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* set Abstract State Feature*/
        for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++)
        {
            g_abstract_state[count] = *(*feature_data + count);
        }
        return USB_OK;
    }

    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Country_Setting
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (COUNTRY_CODE)
 * @param handle: handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Country_Setting(uint32_t handle,
    uint8_t interface,
    uint8_t * *feature_data)
{
    uint8_t count;
    //UNUSED_ARGUMENT (handle)

    /* if interface valid */
    if (interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++)
        {
            g_country_code[count] = *(*feature_data + count);
        }
        return USB_OK;
    }

    return USBERR_INVALID_REQ_TYPE;
}
/*****************************************************************************
 *
 *    @name         APP_init
 *
 *    @brief         This function do initialization for APP.
 *
 *    @param         None
 *
 *    @return       None
 **
 *****************************************************************************/
void TestApp_Init(void)
{
    cdc_config_struct_t cdc_config;
    cdc_config.cdc_application_callback.callback = USB_App_Device_Callback;
    cdc_config.cdc_application_callback.arg = &g_app_handle;
    cdc_config.vendor_req_callback.callback = NULL;
    cdc_config.vendor_req_callback.arg = NULL;
    cdc_config.class_specific_callback.callback = USB_App_Class_Callback;
    cdc_config.class_specific_callback.arg = &g_app_handle;
    cdc_config.desc_callback_ptr = &desc_callback;
    /* Always happen in control endpoint hence hard coded in Class layer*/

    g_cdc_device_speed = USB_SPEED_FULL;
    g_bulk_out_max_packet_size = FS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
    g_bulk_in_max_packet_size = FS_DIC_BULK_IN_ENDP_PACKET_SIZE;
    /* Initialize the USB interface */
    PRINTF("\r\nInitializing CDC...");
    USB_Class_CDC_Init(CONTROLLER_ID, &cdc_config, &g_app_handle);
    PRINTF("OK\r\n");
    g_recv_size = 0;
    g_send_size = 0;
}


/******************************************************************************
 *
 *    @name        USB_App_Device_Callback
 *
 *    @brief       This function handles the callback
 *
 *    @param       handle : handle to Identify the controller
 *    @param       event_type : value of the event
 *    @param       val : gives the configuration value
 *
 *    @return      None
 *
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg)
{
    uint32_t handle;
    handle = *((uint32_t *) arg);
    if (event_type == USB_DEV_EVENT_BUS_RESET)
    {
        start_app = FALSE;
        if (USB_OK == USB_Class_CDC_Get_Speed(handle, &g_cdc_device_speed))
        {
            USB_Desc_Set_Speed(handle, g_cdc_device_speed);
            if (USB_SPEED_HIGH == g_cdc_device_speed)
            {
                g_bulk_out_max_packet_size = HS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
                g_bulk_in_max_packet_size = HS_DIC_BULK_IN_ENDP_PACKET_SIZE;
            }
            else
            {
                g_bulk_out_max_packet_size = FS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
                g_bulk_in_max_packet_size = FS_DIC_BULK_IN_ENDP_PACKET_SIZE;
            }
        }
    }
    else if (event_type == USB_DEV_EVENT_CONFIG_CHANGED)
    {
        /* Schedule buffer for receive */
        USB_Class_CDC_Recv_Data(handle, DIC_BULK_OUT_ENDPOINT, g_curr_recv_buf, g_bulk_out_max_packet_size);
        start_app = TRUE;
    }
    else if (event_type == USB_DEV_EVENT_ERROR)
    {
        /* add user code for error handling */
    }
    return;
}

/******************************************************************************
 *
 *    @name        USB_App_Class_Callback
 *
 *    @brief       This function handles the callback for Get/Set report req
 *
 *    @param       request  :  request type
 *    @param       value    :  give report type and id
 *    @param       data     :  pointer to the data
 *    @param       size     :  size of the transfer
 *
 *    @return      status
 *                  USB_OK  :  if successful
 *                  else return error
 *
 *****************************************************************************/
uint8_t TEST_SEND_BUFFER;

uint8_t USB_App_Class_Callback
(
    uint8_t event,
    uint16_t value,
    uint8_t ** data,
    uint32_t* size,
    void* arg
)
{
	BaseType_t xHigherPriorityTaskWoken; // ms

    cdc_handle_t handle;
    uint8_t error = USB_OK;
    handle = *((cdc_handle_t *) arg);
    switch(event)
    {
    case GET_LINE_CODING:
        error = USB_Get_Line_Coding(handle, value, data);
        break;
    case GET_ABSTRACT_STATE:
        error = USB_Get_Abstract_State(handle, value, data);
        break;
    case GET_COUNTRY_SETTING:
        error = USB_Get_Country_Setting(handle, value, data);
        break;
    case SET_LINE_CODING:
        error = USB_Set_Line_Coding(handle, value, data);
        break;
    case SET_ABSTRACT_STATE:
        error = USB_Set_Abstract_State(handle, value, data);
        break;
    case SET_COUNTRY_SETTING:
        error = USB_Set_Country_Setting(handle, value, data);
        break;
    case USB_APP_CDC_DTE_ACTIVATED:
        if (start_app == TRUE)
        {
            start_transactions = TRUE;
        }
        break;
    case USB_APP_CDC_DTE_DEACTIVATED:
        if (start_app == TRUE)
        {
            start_transactions = FALSE;
        }
        break;
    case USB_DEV_EVENT_DATA_RECEIVED:
        {
        if ((start_app == TRUE) && (start_transactions == TRUE))
        {
            g_recv_size = *size;
        	TEST_SEND_BUFFER = g_curr_recv_buf[0];

        	// Send data to gsm_uart send task
        	if((xQueue_uart_gsm_send_handle != NULL) && (xSemaphore_uart_gsm_send_handle != NULL))
			{
				if(xSemaphoreTakeFromISR(xSemaphore_uart_gsm_send_handle, &xHigherPriorityTaskWoken))
				{
					xQueueSendFromISR( xQueue_uart_gsm_send_handle, &TEST_SEND_BUFFER, &xHigherPriorityTaskWoken );
					//xQueueSend(xQueue_uart_gsm_send_handle, &vc_msg_receive, ( TickType_t ) 5);
					xSemaphoreGive(xSemaphore_uart_gsm_send_handle);
				}
			}
        	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if (!g_recv_size)
            {
                /* Schedule buffer for next receive event */
                USB_Class_CDC_Recv_Data(handle, DIC_BULK_OUT_ENDPOINT, g_curr_recv_buf, g_bulk_out_max_packet_size);
            }
            if ((0 != g_recv_size) && (0xFFFFFFFF != g_recv_size))
            {
            	            g_recv_size = 0;
            	            received_data = TRUE;
            }
        }
    }
        break;
    case USB_DEV_EVENT_SEND_COMPLETE:
        {
        if ((size != NULL) && (*size != 0) && (!(*size % g_bulk_in_max_packet_size)))
        {
            /* If the last packet is the size of endpoint, then send also zero-ended packet,
             ** meaning that we want to inform the host that we do not have any additional
             ** data, so it can flush the output.
             */
            USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT, NULL, 0);
        }
        else if ((start_app == TRUE) && (start_transactions == TRUE))
        {
            if ((*data != NULL) || ((*data == NULL) && (*size == 0)))
            {
                /* User: add your own code for send complete event */
                /* has data been received? */
                if (received_data == TRUE) {
                    /* Schedule buffer for next receive event */
                    received_data = FALSE;
                    USB_Class_CDC_Recv_Data(handle, DIC_BULK_OUT_ENDPOINT, g_curr_recv_buf, g_bulk_out_max_packet_size);
//                    xQueueSend( , ( void * ) &pxMessage, ( TickType_t ) 0 );
                }

//                /* User: add your own code for send complete event */
//                /* Schedule buffer for next receive event */
//                USB_Class_CDC_Recv_Data(handle, DIC_BULK_OUT_ENDPOINT, g_curr_recv_buf, g_bulk_out_max_packet_size);
            }
        }
    }
        break;
    case USB_APP_CDC_SERIAL_STATE_NOTIF:
        {
        /* User: add your own code for serial_state notify event */
    }
        break;
    default:
        {
        error = USBERR_INVALID_REQ_TYPE;
        break;
    }

    }

    return error;
}

/******************************************************************************
 *
 *    @name       Virtual_Com_App
 *
 *    @brief
 *
 *    @param      None
 *
 *    @return     None
 *
 *****************************************************************************/
// Create byte receive buffer
char vc_char_to_send;
char vc_buffer_to_send[256];

void Virtual_Com_App(void)
{
	uint8_t vc_buffer_counter = 0;

	uint8_t error;
	// Receive data from gsm_uart receive task
	if( xQueueReceive( xQueue_virtual_com_send_handle, &vc_char_to_send, ( TickType_t ) 0))
	{
		vc_buffer_to_send[vc_buffer_counter] = vc_char_to_send;
		vc_buffer_counter++;
		vc_buffer_to_send[vc_buffer_counter] = NULL;
		// Empty receive buffer
		while(xQueueReceive( xQueue_virtual_com_send_handle, &vc_char_to_send, ( TickType_t ) 0 ))
		{
			vc_buffer_to_send[vc_buffer_counter] = vc_char_to_send;
			vc_buffer_counter++;
			vc_buffer_to_send[vc_buffer_counter] = NULL;
		}

		error = USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT, vc_buffer_to_send, vc_buffer_counter);
	}
	else
	{
	  // Tell driver there is no data so it can flush output
	  USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT, NULL, 0);
	}
    return;
}

/******************************************************************************
 *
 *    @name       vc_task
 *
 *    @brief
 *
 *    @param      None
 *
 *    @return     None
 *
 *****************************************************************************/
void vc_task(void *arg)
{

	// Protected queue to data send via virtual com
	xSemaphore_virtual_com_send_handle = xSemaphoreCreateMutex();
	xQueue_virtual_com_send_handle = xQueueCreate(1000, sizeof( char ));

	//Receive queue from virtual com ISR
	xQueue_virtual_com_receive_handle = xQueueCreate(300, sizeof( char ));

	// Don't use vc
	while (TRUE)
	{
		vTaskDelay(50000/portTICK_RATE_MS);
	}

	TestApp_Init();


	// Enter while loop for receive message to send
	while (TRUE)
    {
		USB_CDC_Periodic_Task();
		if ((start_app == TRUE) && (start_transactions == TRUE))
		{
			Virtual_Com_App();
		}
		vTaskDelay(5/portTICK_RATE_MS);
    }
}
