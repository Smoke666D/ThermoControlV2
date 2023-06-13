

/*
 * mainFSM.c
 *
 *  Created on: Mar 17, 2023
 *      Author: igor.dymov
 */


#include "mainFSM.h"
#include "mb.h"
#include "../Inc/user_mb_app.h"
#include "din_dout.h"


static EventGroupHandle_t xSystemEventGroupHandle;
static uint8_t mode_restart = 0;
#ifdef MASTER_MODE
	static uint8_t mster_control_addres = 1;
	static uint8_t con_err_count = 0;
	static uint8_t sens_err_count = 0;
	static MASTER_STATE mastersendFSM = BROADCAST_SEND;
	CONECT_ERROR_TYPE connection_error = 0;
	static resisror_errors_t NetStatus[ MAX_SLAVE];
	static uint8_t Dev_Type_Error_Counter = 0;
	CONECT_ERROR_TYPE usGetConnection()
	{
		return (connection_error);
	}
	static TYPE_ERROR_TYPE dev_type_error = 0;
	TYPE_ERROR_TYPE eGetTypeError()
	{
		return (dev_type_error);
	}
	static SENSOR_ERROR_TYPE sens_error;
	SENSOR_ERROR_TYPE eGetSensError()
	{
		return (sens_error);
	}
#endif
#ifdef SLAVE_MODE
static void vSlaveControlFSM();
#endif
static void vMasterControlFSM();
static void vSetState( FAN_SPEED_t speed ,VALVE_STATE_t state );
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;

void InitSystemEnvet(EventGroupHandle_t event)
{
	 xSystemEventGroupHandle = event;
}
void waitFlag( uint32_t flag)
{
	xEventGroupWaitBits(xSystemEventGroupHandle,   flag ,  pdFALSE, pdTRUE, portMAX_DELAY );
}
void setHWInitFlag( uint32_t flag)
{

}
void resetHWInitFlag( uint32_t flag)
{

}



uint8_t temp_error = 0;
static  FAN_SPEED_t current_fan_speed = FAN_SPEED_OFF;
static VALVE_STATE_t  valve_state = VALVE_OFF;
static control_flsm_t control_state = INIT_STATE;
uint16_t input_regs[REG_COUNT];
#ifdef SLAVE_MODE
uint16_t system_regs[DEVICE_HOLDING_FLASG ];
#endif
#ifdef MASTER_MODE
uint16_t system_regs[REG_COUNT ];
#endif
uint8_t connection;


/*REGS_t
 *
 */
uint16_t usGetReg( REGS_t reg_addr)
{
	uint16_t usRes;

     if (reg_addr == MODE)
	 {
			connection = 1;
		}
		usRes = system_regs[reg_addr];

	return  (usRes);
}
/*
 *
 */
uint16_t usGetRegInput( REGS_t reg_addr)
{
	uint16_t usRes;
    usRes = input_regs[reg_addr];
	return  (usRes);
}

void vSetRegInput(REGS_t reg_addr, uint16_t data)
{
	  if (reg_addr == TYPE)
	  {
		  if (input_regs[reg_addr] != data)
		  {
			  mode_restart = 1;
		  }
	  }
      input_regs[reg_addr] = data;
}

void vSetReg(REGS_t reg_addr, uint16_t data)
{
			  if (reg_addr == MODE)
			  {
				  if (system_regs[reg_addr] != data)
				  {
					  mode_restart = 1;
				  }
			  }
			  system_regs[reg_addr] = data;
}


 void vMBTask(void *argument)
 {
#ifdef SLAVE_MODE
	 uint16_t addres = 0;
#endif
	 waitFlag( DIN_READY );

#ifdef SLAVE_MODE
	 addres = (uiGetDinMask() & DEVICE_ADDR_MASK)>>DEVICE_ADDR_OFFSET;
	 eMBInit(MB_RTU,addres,0,19200,MB_PAR_ODD );
	 eMBEnable(  );
#endif
#ifdef MASTER_MODE
	 eMBMasterInit(MB_RTU,0,19200,MB_PAR_ODD );
	 eMBMasterEnable();
#endif
	 xEventGroupSetBits(xSystemEventGroupHandle,  MB_READY );
	 while (1)
	 {

#ifdef SLAVE_MODE
		// vTaskDelay(1);
		 eMBPoll();
#endif
#ifdef MASTER_MODE

		 eMBMasterPoll(  );
#endif
	 }

 }
 uint16_t temp_water;
 uint16_t temp_air;
 uint16_t high_temp;
 uint16_t low_temp;
 dev_type_t device_type = AW;
 dev_mode_t device_mode = DEV_OFF;
 MODE_t mode = OFF_MODE;
 uint8_t door_state = 0;

#ifdef SLAVE_MODE
uint16_t PWM_STATE = 0;
static void vSetPWM( uint16_t pwm)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	if (pwm <=100)
	{
		if (pwm != PWM_STATE)
		{
			 sConfigOC.OCMode = TIM_OCMODE_PWM1;
			  sConfigOC.Pulse = (pwm/100.0)*860;
			  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
			  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
			  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
		}
	}
}


static uint16_t timeout = 0;
static uint16_t timer = 0;
static uint16_t timerR = 0;
#endif



 static void DOUT_PROCESS()
 {


#ifdef SLAVE_MODE
	 uint8_t K2,K3,K1;
	 vSetPWM(usGetReg(PWM));

     if ((usGetRegInput(ERROR_STATUS) &  WATER_TEMP_ERROR ) || ( (usGetRegInput(ERROR_STATUS) & AIR_TEMP_ERROR) && (usGetRegInput(TYPE)==HW)))


     {
    	 timeout= 50;
     }
     else
     {
    	timeout =0;
     }
     if (timeout)
     {
       timer++;
       if (timer>=timeout)
       {
    	 timer =0;
    	 HAL_GPIO_TogglePin( LED_R_GPIO_Port, LED_R_Pin);
       }
     }
     else
     {

    	 HAL_GPIO_WritePin( LED_R_GPIO_Port, LED_R_Pin,GPIO_PIN_RESET);

     }
#endif

#ifdef SLAVE_MODE
	 if (usGetRegInput(TYPE) == NONE)
	 {
		 vUPDATECoils(1);
	 }
	 else
	 {
		 K1 = 0;
		 K2 = 0;
		 K3 = 0;
		 switch(usGetRegInput(FAN_SPEED))
		 {
	     	 case FAN_SPEED_MIN:
	     		 K3 = 1;
	     		 break;
	     	 case FAN_SPEED_MID:

	     		 K2 = 1;
	     		 break;
	     	 case FAN_SPEED_MAX:

	     		 if (usGetRegInput(TYPE) != AW)
	     		 {
	     		   K1 = 1;
	     		 }
	     		 else
	     		 {
	     			 K2 =1;
	     		 }
	     		 break;
	     	default:
	     		break;
		 }
		 vSetOutState( OUT_1, K1 );
	     vSetOutState( OUT_2, K2 );
	     vSetOutState( OUT_3, K3 );
	     vSetOutState( OUT_4, usGetRegInput(WATER_VALVE) );
	 }
#endif
 }

#define HW_DOOR_MASK 0x1
#define AW_DOOR_MASK 0x2

 static void INPUT_PROCESS()
 {

	 if ((uiGetDinMask() & DEVICE_DOOR_MASK)>>DEVICE_DOOR_OFFSET)
	 {
		if (usGetRegInput(DOOR_STATE_TRIGGER)  == OPEN )
		{
			vSetRegInput(DOOR_STATE_TRIGGER,REOPEN);
		}
		else
		{
			vSetRegInput(DOOR_STATE_TRIGGER,OPEN);
		}
	 }

 }


void vDATATask(void *argument)

 {
   //Инициализация типа системы
#ifdef MASTER_MODE
	//uint16_t master_delay = 0;
	for (uint8_t i = 0; i < MAX_SLAVE;i++)
	{
		NetStatus[i].SensorError = 0;
		NetStatus[i].ConnectionError = 0;
	}
#endif
	MAIN_FSM_STATE_t InitFSM = STANDBAY_STATE;
    uint16_t error;

	 while(1)
	 {
		 setHWInitFlag( PROCESS_DATA);
		 vSetRegInput(FSM_STATUS, control_state);
		 vTaskDelay(10);
		 switch (InitFSM)
		 {
		 	 case STANDBAY_STATE:
		 		 //Дожидаемся пока отработают все остальные процессы
		 		 waitFlag( DIN_READY | AIN_READY | MB_READY);
		 		 InitFSM = WORK_STATE;
		 		 mode_restart = 1;
		 		 break;
		 	 case WORK_STATE:

#ifdef SLAVE_MODE
		 		INPUT_PROCESS();
		 		if (mode_restart == 1)
		 		{
		 			vSetRegInput(DOOR_STATE_TRIGGER,CLOSED);
		 			if (usGetRegInput(TYPE)==NONE)
		 			{
		 				control_state = TELEMETRY;
		 			}
		 			else
		 			{
		 			switch (usGetReg(MODE))
		 		    {
		 			  case OFF_MODE:
		 				  vSetState(FAN_SPEED_OFF, VALVE_OFF);
		 				  control_state = STANDBY;
		 				  break;
		 			  case MANUAL_MODE:
		 			  case AUTO_MODE:
		 				 control_state = PREHEAT;
		 			     break;
		 		    }
		 			}
		 			mode_restart = 0;
		 		}
		 		error = usGetRegInput(ERROR_STATUS);
		 		if (((usGetRegInput(TYPE) != NONE) && (error & WATER_TEMP_ERROR ))
		 			|| ((usGetRegInput(TYPE) == HW) && (error & AIR_TEMP_ERROR )))
		 		{
		 			control_state = ERROR_STATE;
		 		}
		 		else
		 		{
		 			if (control_state == ERROR_STATE)
		 			{
		 				mode_restart = 1;
		 				break;
		 			}
		 		}
		 		vSlaveControlFSM();
#endif
#ifdef MASTER_MODE

		 	    vMasterControlFSM();

#endif

		 		DOUT_PROCESS();

		 		break;
		 }
		 resetHWInitFlag( PROCESS_DATA);
	 }
 }

#ifdef SLAVE_MODE
uint32_t sTimer = 0;
void vTimer1sInc()
{
	sTimer++;

}

uint16_t TimerTriger = 0;
 uint8_t GetTimer(uint16_t time)
 {
	uint8_t res = 0;
	if ( TimerTriger == 0 )
	{
		TimerTriger = 1;
		sTimer = 0;
		HAL_TIM_Base_Start_IT(&htim4);
	}
	else
	{
		if (sTimer < time)
		{
			res = 0;
		}
		else
		{
			res = 1;
		}
	}
	return (res);
 }
void ResetTimer()
{
	HAL_TIM_Base_Stop_IT(&htim4);
	TimerTriger = 0;
}
#endif
/*
 *
 */
static void vSetState( FAN_SPEED_t speed ,VALVE_STATE_t state )
{
	if (state != VALVE_AUTO)
	{
		valve_state = state;
	}
	vSetRegInput(WATER_VALVE, valve_state  );

	if ( speed != FAN_SPEED_AUTO)
	{
		current_fan_speed = speed;
	}
	vSetRegInput(FAN_SPEED, current_fan_speed );
}
#ifdef SLAVE_MODE
static void vSlaveControlFSM()
 {
		switch(control_state )
		{
			case ERROR_STATE:
				vSetState(FAN_SPEED_OFF, VALVE_ON);
				break;
			case STANDBY: //Дежурный режим
				if ( usGetRegInput(WATER_TEMP) <=  STANDBY_WATER_ON_TEMP )
				{
					vSetState(FAN_SPEED_OFF, VALVE_ON);
					break;
				}
			    if (usGetRegInput(WATER_TEMP) >=   STANDBY_WATER_OFF_TEMP )
				{
			    	vSetState(FAN_SPEED_OFF, VALVE_OFF);
			    	break;
				}
			    vSetState(FAN_SPEED_OFF, VALVE_AUTO);
				break;
			case PREHEAT:
				    vSetState(FAN_SPEED_OFF, VALVE_ON);
				    if ( ( usGetReg(MODE) == DEV_MANUAL) && (usGetRegInput(TYPE) == HWC))
				    {
				    	control_state = (usGetRegInput(WATER_TEMP) > 8 ) ? INIT_WORK : PREHEAT;
				    }
				    else
				    {
				    	if (GetTimer(PREHEAT_OFF_TIME) &&  (usGetRegInput(WATER_TEMP) >= PREHEAT_OFF_TEMP ) )
				    	{
				    		ResetTimer();
				    		control_state =  INIT_WORK;
				    	}
				    }
					break;
			case INIT_WORK:
				vSetRegInput(DOOR_STATE_TRIGGER,CLOSED);
				control_state =  WORK;
				if  ((usGetRegInput(TYPE) == HW) && ( usGetReg(MODE) == DEV_AUTO))
				{
					if ( usGetRegInput(IN_AIR_TEMP) < ( usGetReg(WORK_TEMP) - 2*TEMP_DELTA))
					{
						vSetState(FAN_SPEED_MAX, VALVE_ON);
						break;
					}
					if (usGetRegInput(IN_AIR_TEMP) < ( usGetReg(WORK_TEMP) - TEMP_DELTA))
					{
						vSetState(FAN_SPEED_MID,VALVE_ON);
						break;
					}
					if  (usGetRegInput(IN_AIR_TEMP)  <  usGetReg(WORK_TEMP))
					{
						vSetState(FAN_SPEED_MIN ,VALVE_ON);
						break;
					}
					if (usGetRegInput(IN_AIR_TEMP) ==  usGetReg(WORK_TEMP) )
					{
						vSetState(FAN_SPEED_MIN ,VALVE_OFF);
						break;
					}
					if (usGetRegInput(IN_AIR_TEMP)  >  usGetReg(WORK_TEMP) )
					{
						vSetState(FAN_SPEED_OFF ,VALVE_OFF);
						break;
					}
				}
				if ( (usGetRegInput(TYPE) == AW) && ( usGetReg(MODE) == DEV_AUTO))
				{
					if (usGetReg(AIR_TEMP) < ( usGetReg(WORK_TEMP) - TEMP_DELTA))
					{
						vSetState( FAN_SPEED_MID, VALVE_ON);
						break;
					}
					if (usGetReg(AIR_TEMP) <  usGetReg(WORK_TEMP))
					{
						vSetState( FAN_SPEED_MIN ,VALVE_ON);
						break;
					}
					if (usGetReg(AIR_TEMP) ==  usGetReg(WORK_TEMP))
					{
						vSetState( FAN_SPEED_MIN ,VALVE_OFF );
						break;
					}
					if (usGetReg(AIR_TEMP) > usGetReg(WORK_TEMP) )
					{
						vSetState( FAN_SPEED_OFF ,VALVE_OFF );
					    break;
					}
				}
				if (( usGetReg(MODE) == DEV_MANUAL) && (usGetRegInput(TYPE) == HWC))
				{
					vSetState(usGetReg(FAN_SPEED_CONFIG),( usGetReg(AIR_TEMP) <  usGetReg(WORK_TEMP)) ? VALVE_OFF: VALVE_ON);
				}
				else
				{
					vSetState(usGetReg(FAN_SPEED_CONFIG),( usGetReg(AIR_TEMP) <  usGetReg(WORK_TEMP)) ? VALVE_ON: VALVE_OFF);
				}

				break;
				 case WORK:
					    if (( usGetReg(MODE) == DEV_MANUAL) && (usGetRegInput(TYPE) == HWC))
						{
					    	if (usGetRegInput(WATER_TEMP) < 5)
					    	{
					    		control_state = PREHEAT;
					    		break;
					    	}
						}
					    else
					    {
					    	//Режим разморозки
					    	if (usGetRegInput(WATER_TEMP) < WATER_FREEZE_TEMP)
					    	{
					    		control_state = PREHEAT;
					    		break;
					    	}
					    }
					    //Режим срабатывания дверных концевиков
					    if ( (usGetRegInput(DOOR_STATE_TRIGGER) !=CLOSED) &&
					    		( (usGetReg(MODE) == DEV_AUTO ) ||  ((usGetReg(MODE) == DEV_MANUAL ) && (usGetRegInput(TYPE) == HWC))))
						{
					    	if (usGetRegInput(DOOR_STATE_TRIGGER) == REOPEN)
					    	{
					    		 vSetRegInput(DOOR_STATE_TRIGGER,OPEN);
					    		 ResetTimer();
					    	}
					    	if (((uiGetDinMask() & DEVICE_DOOR_MASK)>>DEVICE_DOOR_OFFSET ) == 0)
							 {
								if (GetTimer(DOOR_CLOSE_TIME))
								{
									 ResetTimer();
									 control_state =  INIT_WORK;
									 break;
								}
							}
							vSetState(FAN_SPEED_MAX, VALVE_ON);
							break;
						}
					    if ((usGetRegInput(TYPE) == HW) && ( usGetReg(MODE) == DEV_AUTO))
					    {
					    	if ( usGetRegInput(IN_AIR_TEMP)  <= ( usGetReg(WORK_TEMP) - 3*TEMP_DELTA  ) )
					    	{
					    		vSetState(FAN_SPEED_MAX, VALVE_ON);
					    		break;
					    	}
					    	if ( usGetRegInput(IN_AIR_TEMP) < ( usGetReg(WORK_TEMP) - 2*TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_AUTO, VALVE_ON);
					    		break;
					    	}
					    	if ( usGetRegInput(IN_AIR_TEMP) == ( usGetReg(WORK_TEMP) - 2*TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_MID, VALVE_ON);
					    	    break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP) < ( usGetReg(WORK_TEMP) - TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_AUTO,VALVE_ON);
					    		break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP) == ( usGetReg(WORK_TEMP) - TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_MIN ,VALVE_ON);
					    		break;
					    	 }
					    	if  (usGetRegInput(IN_AIR_TEMP)  <  usGetReg(WORK_TEMP))
					    	{
					    		vSetState(FAN_SPEED_AUTO ,VALVE_AUTO);
					    		break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP) ==  usGetReg(WORK_TEMP) )
					    	{
					    		vSetState(FAN_SPEED_MIN ,VALVE_OFF);
					    		break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP)  < ( usGetReg(WORK_TEMP) + TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_AUTO ,VALVE_OFF);
					    		break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP) >= ( usGetReg(WORK_TEMP) + TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_OFF,VALVE_OFF);
					    		break;
					    	}
					    }
					    if (( usGetRegInput(TYPE) == HWC ) && ( usGetReg(MODE) == DEV_MANUAL))
					 	{
					 		if (usGetReg(AIR_TEMP) > ( usGetReg(WORK_TEMP) + VALVE_OFF_TEMP_DELTA))
					 		{
					 			vSetState(usGetReg(FAN_SPEED_CONFIG), VALVE_ON);
					 			break;
					 		}
					 		if (usGetReg(AIR_TEMP) < (usGetReg(WORK_TEMP) - VALVE_ON_TEMP_DELTA ))
					 		{
					 			vSetState(usGetReg(FAN_SPEED_CONFIG), VALVE_OFF);
					 			break;
					 		}
					 		vSetState(usGetReg(FAN_SPEED_CONFIG), VALVE_AUTO);
					 		break;
					 	}

					    if (( usGetRegInput(TYPE) == HWC ) && ( usGetReg(MODE) == DEV_AUTO))
					   	{
					   			if (usGetReg(AIR_TEMP) > ( usGetReg(WORK_TEMP) + VALVE_OFF_TEMP_DELTA))
					   			{
					   					 vSetState(usGetReg(FAN_SPEED_CONFIG), VALVE_OFF);
					   					 break;
					   			}
					   			if (usGetReg(AIR_TEMP) < (usGetReg(WORK_TEMP) - VALVE_ON_TEMP_DELTA ))
					   			{
					   					vSetState(usGetReg(FAN_SPEED_CONFIG), VALVE_ON);
					   					 break;
					   			}
					   			vSetState(usGetReg(FAN_SPEED_CONFIG), VALVE_AUTO);
					   			break;
					   	}
					    if  ( usGetReg(MODE) == DEV_MANUAL)
					    {
					    	if (usGetReg(AIR_TEMP) > ( usGetReg(WORK_TEMP) + VALVE_OFF_TEMP_DELTA))
					    	{
					    		vSetState(usGetReg(FAN_SPEED_CONFIG), VALVE_OFF);
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) < (usGetReg(WORK_TEMP) - VALVE_ON_TEMP_DELTA ))
					    	{
					    		vSetState(usGetReg(FAN_SPEED_CONFIG), VALVE_ON);
					    		break;
					    	}
					    	vSetState(usGetReg(FAN_SPEED_CONFIG), VALVE_AUTO);
					    	break;
					    }
					    if ((usGetRegInput(TYPE) == AW) && ( usGetReg(MODE) == DEV_AUTO))
					    {
					    	if (usGetReg(AIR_TEMP) <= (usGetReg(WORK_TEMP) - TEMP_DELTA*2 ))
					    	{
					    		vSetState(FAN_SPEED_MID, VALVE_ON);
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) < (usGetReg(WORK_TEMP) - TEMP_DELTA ))
					    	{
					    		vSetState(FAN_SPEED_AUTO, VALVE_ON);
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) == ( usGetReg(WORK_TEMP) - TEMP_DELTA))
					    	{
					    		vSetState( FAN_SPEED_MIN, VALVE_ON);
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) <  usGetReg(WORK_TEMP))
					    	{
					    		vSetState( FAN_SPEED_MIN ,VALVE_AUTO);
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) ==  usGetReg(WORK_TEMP))
					    	{
					    		vSetState( FAN_SPEED_MIN ,VALVE_OFF );
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) < (usGetReg(WORK_TEMP) + TEMP_DELTA))
					    	{
					    		vSetState( FAN_SPEED_AUTO ,VALVE_OFF );
					    		break;
					        }
					    	if (usGetReg(AIR_TEMP) >= (usGetReg(WORK_TEMP) + TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_OFF,VALVE_OFF );
					    		break;
					    	}
					    }
					 break;
				 default:
					 break;
			}
 }
#endif
#ifdef MASTER_MODE



void vMasterControlFSM()
{
	eMBMasterReqErrCode    errorCode = MB_MRE_NO_ERR;
    uint16_t mode;
    uint16_t errors;
	uint16_t data[4];
    switch(usGetReg(MODE) )
    {
    	case 0:
    		data[0]= OFF_MODE;
    		break;
    	case 1:
    		data[0] = AUTO_MODE;
    		break;
    	case 2:
    		data[0] = MANUAL_MODE;
    		break;
    }

	switch (usGetReg(FAN_SPEED_CONFIG))
	{
		case 0:
			data[1] = FAN_SPEED_MID;
			break;
		case 2:
			data[1] = FAN_SPEED_MIN;
			break;
		case 1:
			data[1] = FAN_SPEED_MAX;
			break;
	}
	data[2]=usGetReg(WORK_TEMP);
	data[3]=usGetReg(AIR_TEMP);

	switch (mastersendFSM)
	{
			case 0:
				mastersendFSM =  ( eMBMasterReqWriteMultipleHoldingRegister( 0, 13, 4, &data[0], 0)  == MB_MRE_NO_ERR )  ?  ADRESS_SEND : BROADCAST_SEND;
				break;
			case 1:
				if (  usGetReg( CONTROL_MODE )  )
				{

					errorCode = eMBMasterReqReadInputRegister( mster_control_addres, 5, 8, 0 );
					switch (errorCode)
					{
						case MB_MRE_NO_REG:                  /*!< illegal register address. */
						case MB_MRE_ILL_ARG:                 /*!< illegal argument. */
						case MB_MRE_EXE_FUN:
					    case MB_MRE_TIMEDOUT:
						case MB_MRE_REV_DATA:
							  	  	  	NetStatus[mster_control_addres].ConnectionError++;
						case MB_MRE_NO_ERR:
									    if ( errorCode == MB_MRE_NO_ERR)
									    {
									    	NetStatus[mster_control_addres].ConnectionError = 0;
											mode =   usGetInput( mster_control_addres-1, 0 );
											errors = usGetInput( mster_control_addres-1, 7 );
											if  ( mode !=  usGetReg(DEVICE_TYPE)) Dev_Type_Error_Counter++;
											if   ( (mode == HW) || (mode == TELEMETRY ) )
											{
												if ( errors & AIR_TEMP_ERROR ) sens_err_count++;
											}
										    if ( errors & WATER_TEMP_ERROR ) sens_err_count++;
										}
											  //Порверяем счетчик ошибок подключения
											  if ( NetStatus[mster_control_addres].ConnectionError  >= MAX_CONNECTION_ERROR)
											  {
												  //Если счетчик переполнее, то устанваливаем его на значение срабатывания, чтобы он не переполнился
												  NetStatus[mster_control_addres].ConnectionError = MAX_CONNECTION_ERROR;
												  //Устанавливаем флаг ошибок подлючения
												  con_err_count++;
											  }
											  if ( ++mster_control_addres > usGetReg( DEVICE_COUNT ) )
											  {
												   mster_control_addres = 1;
												   connection_error = (con_err_count == 0) ? NO_CONNECTION_ERROR : CONNECTION_ERROR_PERESENT;
												   con_err_count = 0;
												   dev_type_error = ( Dev_Type_Error_Counter != 0 ) ? TYPE_ERROR : NO_TYPE_ERROR;
												   Dev_Type_Error_Counter = 0;
												   sens_error = (sens_err_count!= 0) ? SENSOR_ERROR : NO_SENSOR_ERROR;
												   sens_err_count = 0;
												}
												mastersendFSM = BROADCAST_SEND;
												break;
						default:

							break;
					}
				}
				else
				{
					mastersendFSM = 0;
				}
				break;
	}
}
#endif
