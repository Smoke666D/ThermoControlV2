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
 * File: $Id: portevent_m.c v 1.60 2013/08/13 15:07:05 Armink add Master Functions$
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "../Inc/mb.h"
#include "../Inc/mb_m.h"
#include "../Inc/mbport.h"
#include "../Inc/port.h"
#include "mainFSM.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Defines ------------------------------------------*/
/* ----------------------- Variables ----------------------------------------*/

static EventGroupHandle_t xMasterOsEventGroupHandle;
static SemaphoreHandle_t xSem;
//static struct rt_semaphore xMasterRunRes;
//static struct rt_event     xMasterOsEvent;
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBMasterPortEventInit( void )
{
	xMasterOsEventGroupHandle = xGetOSEvent();
   // rt_event_init(&xMasterOsEvent,"master event",RT_IPC_FLAG_PRIO);
    return TRUE;
}

BOOL
xMBMasterPortEventPost( eMBMasterEventType eEvent )
{
	BaseType_t xHigherPriorityTaskWoken, xResult;
	eMBMasterEventType xEvent = eEvent;

	    switch (xEvent)
		{
		    case EV_MASTER_READY_ISR:
		    case EV_MASTER_FRAME_RECEIVED_ISR:
		    case EV_MASTER_ERROR_PROCESS_ISR:
		    case EV_MASTER_EXECUTE_ISR:

		    		/* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
		    		xHigherPriorityTaskWoken = pdFALSE;

		    		 /* Set bit 0 and bit 4 in xEventGroup. */
		    		 if (xEvent == EV_MASTER_EXECUTE_ISR)
		    		 {
		    			 xEvent = EV_MASTER_EXECUTE;
		    		 }
		    		 if (xEvent == EV_MASTER_ERROR_PROCESS_ISR)
		    		 {
		    				   xEvent = EV_MASTER_ERROR_PROCESS;
		    		 }
		    		 xResult = xEventGroupSetBitsFromISR(
		    				 xMasterOsEventGroupHandle,   /* The event group being updated. */
													 xEvent , /* The bits being set. */
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

		        break;
		    case EV_MASTER_ERROR_PROCESS:
		    case EV_MASTER_ERROR_RESPOND_TIMEOUT:
		    case EV_MASTER_ERROR_RECEIVE_DATA:
		    case EV_MASTER_ERROR_EXECUTE_FUNCTION:
		    case EV_MASTER_EXECUTE:
		    case EV_MASTER_FRAME_SENT:
		    case EV_MASTER_PROCESS_SUCESS:
		    	xEventGroupSetBits(xMasterOsEventGroupHandle,xEvent);
		        break;
		    default:
		    	break;
		}
 //   rt_event_send(&xMasterOsEvent, eEvent);
    return (TRUE);
}

BOOL
xMBMasterPortEventGet( eMBMasterEventType * eEvent )
{

	 EventBits_t recvedEvent;
	 recvedEvent = xEventGroupWaitBits(xMasterOsEventGroupHandle,    EV_MASTER_READY_ISR | EV_MASTER_FRAME_RECEIVED_ISR | EV_MASTER_EXECUTE |
	            EV_MASTER_FRAME_SENT | EV_MASTER_ERROR_PROCESS,   pdTRUE, pdFALSE, portMAX_DELAY );
 //   rt_uint32_t recvedEvent;
    /* waiting forever OS event */
  /*  rt_event_recv(&xMasterOsEvent,
            EV_MASTER_READY | EV_MASTER_FRAME_RECEIVED | EV_MASTER_EXECUTE |
            EV_MASTER_FRAME_SENT | EV_MASTER_ERROR_PROCESS,
            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
            &recvedEvent);
    /the enum type couldn't convert to int type */
    switch (recvedEvent)
    {
    case EV_MASTER_READY_ISR:
        *eEvent = EV_MASTER_READY_ISR;
        break;
    case EV_MASTER_FRAME_RECEIVED_ISR:
        *eEvent = EV_MASTER_FRAME_RECEIVED_ISR;
        break;
    case EV_MASTER_EXECUTE:
        *eEvent = EV_MASTER_EXECUTE;
        break;
    case EV_MASTER_FRAME_SENT:
        *eEvent = EV_MASTER_FRAME_SENT;
        break;
    case EV_MASTER_ERROR_PROCESS:
        *eEvent = EV_MASTER_ERROR_PROCESS;
        break;
    }
    return TRUE;
}
/**
 * This function is initialize the OS resource for modbus master.
 * Note:The resource is define by OS.If you not use OS this function can be empty.
 *
 */
void vMBMasterOsResInit( void )
{
	xSem = xGetSystemSem();
   // rt_sem_init(&xMasterRunRes, "master res", 0x01 , RT_IPC_FLAG_PRIO);
}

/**
 * This function is take Mobus Master running resource.
 * Note:The resource is define by Operating System.If you not use OS this function can be just return TRUE.
 *
 * @param lTimeOut the waiting time.
 *
 * @return resource taked result
 */
BOOL xMBMasterRunResTake( LONG lTimeOut )
{
	//xSemaphoreTake(xSem, lTimeOut );
    //If waiting time is -1 .It will wait forever
    return xSemaphoreTake(xSem, portTICK_PERIOD_MS*lTimeOut  ) ? FALSE: TRUE ;
	//return TRUE;
}

/**
 * This function is release Mobus Master running resource.
 * Note:The resource is define by Operating System.If you not use OS this function can be empty.
 *
 */
void vMBMasterRunResRelease( void )
{
    /* release resource */
	xSemaphoreGive(xSem);
   // rt_sem_release(&xMasterRunRes);
}

/**
 * This is modbus master respond timeout error process callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 * @param ucDestAddress destination salve address
 * @param pucPDUData PDU buffer data
 * @param ucPDULength PDU buffer length
 *
 */
void vMBMasterErrorCBRespondTimeout(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) {
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
   // rt_event_send(&xMasterOsEvent, EV_MASTER_ERROR_RESPOND_TIMEOUT);
	xEventGroupSetBits(xMasterOsEventGroupHandle,EV_MASTER_ERROR_RESPOND_TIMEOUT);
    /* You can add your code under here. */

}

/**
 * This is modbus master receive data error process callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 * @param ucDestAddress destination salve address
 * @param pucPDUData PDU buffer data
 * @param ucPDULength PDU buffer length
 *
 */
void vMBMasterErrorCBReceiveData(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) {
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
  //  rt_event_send(&xMasterOsEvent, EV_MASTER_ERROR_RECEIVE_DATA);
	xEventGroupSetBits(xMasterOsEventGroupHandle,EV_MASTER_ERROR_RECEIVE_DATA);
    /* You can add your code under here. */

}

/**
 * This is modbus master execute function error process callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 * @param ucDestAddress destination salve address
 * @param pucPDUData PDU buffer data
 * @param ucPDULength PDU buffer length
 *
 */
void vMBMasterErrorCBExecuteFunction(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) {
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
   // rt_event_send(&xMasterOsEvent, EV_MASTER_ERROR_EXECUTE_FUNCTION);
	xEventGroupSetBits(xMasterOsEventGroupHandle,EV_MASTER_ERROR_EXECUTE_FUNCTION);
    /* You can add your code under here. */

}

/**
 * This is modbus master request process success callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 */
void vMBMasterCBRequestScuuess( void ) {
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
  //  rt_event_send(&xMasterOsEvent, EV_MASTER_PROCESS_SUCESS);
	xEventGroupSetBits(xMasterOsEventGroupHandle,EV_MASTER_PROCESS_SUCESS);
    /* You can add your code under here. */

}

/**
 * This function is wait for modbus master request finish and return result.
 * Waiting result include request process success, request respond timeout,
 * receive data error and execute function error.You can use the above callback function.
 * @note If you are use OS, you can use OS's event mechanism. Otherwise you have to run
 * much user custom delay for waiting.
 *
 * @return request error code
 */
eMBMasterReqErrCode eMBMasterWaitRequestFinish( void ) {
    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
 //   rt_uint32_t recvedEvent;
    /* waiting for OS event */

    EventBits_t recvedEvent;
   	 recvedEvent = xEventGroupWaitBits(xMasterOsEventGroupHandle,   EV_MASTER_PROCESS_SUCESS | EV_MASTER_ERROR_RESPOND_TIMEOUT
             | EV_MASTER_ERROR_RECEIVE_DATA
             | EV_MASTER_ERROR_EXECUTE_FUNCTION,   pdTRUE, pdFALSE, portMAX_DELAY );
   /* rt_event_recv(&xMasterOsEvent,
            EV_MASTER_PROCESS_SUCESS | EV_MASTER_ERROR_RESPOND_TIMEOUT
                    | EV_MASTER_ERROR_RECEIVE_DATA
                    | EV_MASTER_ERROR_EXECUTE_FUNCTION,
            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
            &recvedEvent);
            */
    switch (recvedEvent)
    {
    case EV_MASTER_PROCESS_SUCESS:
        break;
    case EV_MASTER_ERROR_RESPOND_TIMEOUT:
    {
        eErrStatus = MB_MRE_TIMEDOUT;
        break;
    }
    case EV_MASTER_ERROR_RECEIVE_DATA:
    {
        eErrStatus = MB_MRE_REV_DATA;
        break;
    }
    case EV_MASTER_ERROR_EXECUTE_FUNCTION:
    {
        eErrStatus = MB_MRE_EXE_FUN;
        break;
    }
    default:
    	eErrStatus = 55;
    	break;
    }
    return eErrStatus;
}

#endif
