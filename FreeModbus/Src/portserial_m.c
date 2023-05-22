/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions $
 */

#include "../Inc/mb.h"
#include "../Inc/mbport.h"
#include "../Inc/port.h"
#include "mainFSM.h"


extern UART_HandleTypeDef huart1;
#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Static variables ---------------------------------*/
static EventGroupHandle_t xSerialEventGroupHandle;
static uint8_t rx_data_buf;
static uint8_t rx_data;
//ALIGN(RT_ALIGN_SIZE)
/* software simulation serial transmit IRQ handler thread stack */
//static rt_uint8_t serial_soft_trans_irq_stack[512];
/* software simulation serial transmit IRQ handler thread */
//static struct rt_thread thread_serial_soft_trans_irq;
/* serial event */
//static struct rt_event event_serial;
/* modbus master serial device */
//static rt_serial_t *serial;

/* ----------------------- Defines ------------------------------------------*/
/* serial transmit event */
#define EVENT_SERIAL_TRANS_START    (1<<0)

/* ----------------------- static functions ---------------------------------*/
//static void prvvUARTTxReadyISR(void);
//static void prvvUARTRxISR(void);
//static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size);
//static void serial_soft_trans_irq(void* parameter);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity)
{
	xSerialEventGroupHandle = xGetUARTEvent();
	/**
     * set 485 mode receive and transmit control IO
     * @note MODBUS_MASTER_RT_CONTROL_PIN_INDEX need be defined by user
     */
   // rt_pin_mode(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_MODE_OUTPUT);

    huart1.Init.BaudRate = ulBaudRate;
     huart1.Init.StopBits = UART_STOPBITS_1;

    switch(eParity)
    {
    	case MB_PAR_NONE:
    		huart1.Init.WordLength = UART_WORDLENGTH_8B;
    		huart1.Init.Parity = UART_PARITY_NONE;
    		break;
    	case MB_PAR_ODD:
    		huart1.Init.WordLength = UART_WORDLENGTH_9B;
    	  	huart1.Init.Parity = UART_PARITY_ODD;
    	  	break;
    	case MB_PAR_EVEN:
    		huart1.Init.WordLength = UART_WORDLENGTH_9B;
    		huart1.Init.Parity = UART_PARITY_EVEN;
    		break;
    }
    /* set serial configure */

    HAL_UART_Init(&huart1);


    /* open serial device */
    //if (!serial->parent.open(&serial->parent,
     //       RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX )) {
      //  serial->parent.rx_indicate = serial_rx_ind;
   // } else {
    //    return FALSE;
   // }

    /* software initialize */
   // rt_event_init(&event_serial, "master event", RT_IPC_FLAG_PRIO);
   // rt_thread_init(&thread_serial_soft_trans_irq,
    //               "master trans",
     //              serial_soft_trans_irq,
      //             RT_NULL,
       //            serial_soft_trans_irq_stack,
        //           sizeof(serial_soft_trans_irq_stack),
         //          10, 5);
    //rt_thread_startup(&thread_serial_soft_trans_irq);

    return TRUE;
}

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
 //   rt_uint32_t recved_event;
    if (xRxEnable)
    {
    	HAL_UART_Receive_IT(&huart1,&rx_data_buf,1);
        HAL_HalfDuplex_EnableReceiver(&huart1);
        /* enable RX interrupt */
   //     serial->ops->control(serial, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        /* switch 485 to receive mode */
  //      rt_pin_write(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_LOW);
    }
    else
    {
    	HAL_UART_AbortReceive_IT(&huart1);
        /* switch 485 to transmit mode */
    //    rt_pin_write(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_HIGH);
        /* disable RX interrupt */
      //  serial->ops->control(serial, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_RX);
    }
    if (xTxEnable)
    {
        /* start serial transmit */
    	HAL_HalfDuplex_EnableTransmitter(&huart1);
    	xEventGroupSetBits(xSerialEventGroupHandle,EVENT_SERIAL_TRANS_START);
       // rt_event_send(&event_serial, EVENT_SERIAL_TRANS_START);
    }
    else
    {
        /* stop serial transmit */
    	xEventGroupWaitBits( xSerialEventGroupHandle,  EVENT_SERIAL_TRANS_START,  pdTRUE, pdFALSE, 0 );
      //  rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
       //         RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0,
        //        &recved_event);
    }
}

void vMBMasterPortClose(void)
{
   // serial->parent.close(&(serial->parent));
}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
	HAL_UART_Transmit_IT(&huart1,(uint8_t *) &ucByte, 1);
  //  serial->parent.write(&(serial->parent), 0, &ucByte, 1);
    return TRUE;
}

BOOL xMBMasterPortSerialGetByte(CHAR * pucByte)
{
	* pucByte = rx_data;
 //   serial->parent.read(&(serial->parent), 0, pucByte, 1);
    return TRUE;
}


#ifdef MASTER_MODE

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

	BaseType_t xHigherPriorityTaskWoken, xResult;
	/* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
	xHigherPriorityTaskWoken = pdFALSE;

	 /* Set bit 0 and bit 4 in xEventGroup. */
	 xResult = xEventGroupSetBitsFromISR(
			 	 	 	 	 	   xSerialEventGroupHandle,   /* The event group being updated. */
								   EVENT_SERIAL_TRANS_START, /* The bits being set. */
	                              &xHigherPriorityTaskWoken );

	  /* Was the message posted successfully? */
	  if( xResult != pdFAIL )
	  {
	      /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
	      switch should be requested.  The macro used is port specific and will
	      be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
	      the documentation page for the port being used. */
	      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	  }
}



void StartUARTTask(void *argument)
{
	  EventBits_t uxBits;
	  while(1)
	  {
		  uxBits = xEventGroupWaitBits( xSerialEventGroupHandle,  EVENT_SERIAL_TRANS_START,  pdTRUE, pdFALSE, portMAX_DELAY );
		  switch (uxBits)
		  {
		  	  	  case EVENT_SERIAL_TRANS_START:
		  	  		 pxMBMasterFrameCBTransmitterEmpty();

		  	  		  break;
		  	  	  default:
		  	  		  break;
		  }
	  }


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rx_data = rx_data_buf;
	HAL_UART_Receive_IT(&huart1,&rx_data_buf,1);
	pxMBMasterFrameCBByteReceived();
}

#endif

/*
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
//void prvvUARTTxReadyISR(void)
//{
   // pxMBMasterFrameCBTransmitterEmpty();
//}

/*
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
//void prvvUARTRxISR(void)
//{
  //  pxMBMasterFrameCBByteReceived();
//}

/**
 * Software simulation serial transmit IRQ handler.
 *
 * @param parameter parameter
 */


/**
 * This function is serial receive callback function
 *
 * @param dev the device of serial
 * @param size the data size that receive
 *
 * @return return RT_EOK
 */
//static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size) {
 //   prvvUARTRxISR();
 //   return RT_EOK;
//}

#endif
