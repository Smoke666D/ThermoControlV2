

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

uint8_t connection_error = 0;
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
				  mode_restart = 1;
			  }
			  system_regs[reg_addr] = data;
}


 void vMBTask(void *argument)
 {
	 uint16_t addres = 0;
	 waitFlag( DIN_READY );

#ifdef SLAVE_MODE
	 addres = (uiGetDinMask() & DEVICE_ADDR_MASK)>>DEVICE_ADDR_OFFSET;
	 eMBInit(MB_RTU,addres,0,115200,MB_PAR_ODD );
	 eMBEnable(  );
#endif
#ifdef MASTER_MODE
	 eMBMasterInit(MB_RTU,0,115200,MB_PAR_ODD );
	 eMBMasterEnable();
#endif
	 xEventGroupSetBits(xSystemEventGroupHandle,  MB_READY );
	 while (1)
	 {
		 vTaskDelay(1);
#ifdef SLAVE_MODE
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
#endif

static uint16_t timeout = 0;
static uint16_t timer = 0;
static uint16_t timerR = 0;

 static void DOUT_PROCESS()
 {
#ifdef MASTER_MODE
	 if (usGetRegInput(ERROR_STATUS) &  AIR_TEMP_ERROR )
	 {
	 		timeout = 50;
	 }
	 else
	 {
	    if ( connection_error )
	    {
	    	timeout = 25;
	    }
	    else
	    {
	     		timeout = 0;
	    }
	 }
	 if ( timeout )
	     {
	     	timer++;
	     	if ( timer >= timeout )
	     	{
	     		timer = 0U;
	     		HAL_GPIO_TogglePin( LED_G_GPIO_Port, LED_G_Pin);
	     	}
	     }
	     else
	     {
	    	 HAL_GPIO_WritePin( LED_G_GPIO_Port, LED_G_Pin, (usGetReg(MODE) != OFF_MODE) ? GPIO_PIN_RESET: GPIO_PIN_SET);

	     }
	     switch ( usGetReg(MODE)  )
	     {
	     	case OFF_MODE:

	     		HAL_GPIO_WritePin( LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET );
	     		break;
	     	case MANUAL_MODE:
	     		if (usGetRegInput(TYPE) == HWC)
	     		{
	     			HAL_GPIO_WritePin( LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET );
	     		}
	     		else
	     		{
	     			if (usGetReg(WORK_TEMP) < usGetReg(AIR_TEMP))
	     			{
	     				HAL_GPIO_WritePin( LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET );
	     			}
	     			else
	     			{
	     				HAL_GPIO_WritePin( LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET );
	     			}
	     		}
	     		break;
	     	case AUTO_MODE:
	     		timerR++;
	     		if (timerR>=50)
	     		{
	     		    timerR =0;
	     		    HAL_GPIO_TogglePin( LED_R_GPIO_Port, LED_R_Pin);
	     		}
	     		break;
	     }

#endif

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
	uint16_t master_delay = 0;
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
		 		INPUT_PROCESS();
#ifdef SLAVE_MODE
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
		 				  vSetState(FAN_SPEED_OFF, VALVE_ON);
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
		 			vSetState(FAN_SPEED_OFF, VALVE_ON);
		 		}
		 		else
		 		{
		 			vSlaveControlFSM();
		 		}
#endif
#ifdef MASTER_MODE
		 		if ( ++master_delay == 7)
		 		{
		 			master_delay = 0;
		 			vMasterControlFSM();
		 		}
#endif

		 		DOUT_PROCESS();

		 		break;
		 }
		 resetHWInitFlag( PROCESS_DATA);
	 }
 }


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
					 if (GetTimer(PREHEAT_OFF_TIME) &&  (usGetRegInput(WATER_TEMP) >= PREHEAT_OFF_TEMP ) )
					 {
						 ResetTimer();
						 control_state =  WORK;
					 }
					 break;
				 case WORK:
					    //Режим разморозки
					    if (usGetRegInput(WATER_TEMP) < WATER_FREEZE_TEMP)
					 	{
					    	vSetRegInput(DOOR_STATE_TRIGGER,CLOSED);
					 		control_state = PREHEAT;
					 		break;
					 	}
					    //Режим срабатывания дверных концевиков
					    if ((usGetRegInput(DOOR_STATE_TRIGGER) !=CLOSED) && (usGetReg(MODE) == DEV_AUTO ))
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
									 vSetRegInput(DOOR_STATE_TRIGGER,CLOSED);
									 ResetTimer();
								}
							}
							else
							{
								vSetState(FAN_SPEED_MAX, VALVE_ON);
							}
							break;
						}
					    if ((usGetRegInput(TYPE) == HW) && ( usGetReg(MODE) == DEV_AUTO))
					    {

					    	if ( usGetRegInput(IN_AIR_TEMP)  < ( usGetReg(WORK_TEMP) - SPEED_3_HW_SWITCH_TEMP_DELTA  ) )
					    	{
					    		vSetState(FAN_SPEED_MAX, VALVE_ON);
					    		break;
					    	}
					    	if ( usGetRegInput(IN_AIR_TEMP) < ( usGetReg(WORK_TEMP) - SPEED_2_HW_SWITCH_TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_AUTO,  VALVE_AUTO);
					    		break;
					    	}
					    	if ( usGetRegInput(IN_AIR_TEMP) == ( usGetReg(WORK_TEMP) - SPEED_2_HW_SWITCH_TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_MID, VALVE_AUTO);
					    	    break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP) < ( usGetReg(WORK_TEMP) - SPEED_1_HW_SWITCH_TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_AUTO,VALVE_AUTO);
					    		break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP) == ( usGetReg(WORK_TEMP) - SPEED_1_HW_SWITCH_TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_MIN ,VALVE_AUTO);
					    		break;
					    	 }
					    	if  (usGetRegInput(IN_AIR_TEMP)  <  usGetReg(WORK_TEMP))
					    	{
					    		vSetState(FAN_SPEED_AUTO ,VALVE_AUTO);
					    		break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP) ==  usGetReg(WORK_TEMP) )
					    	{
					    		vSetState(FAN_SPEED_AUTO ,VALVE_OFF);
					    		break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP)  < ( usGetReg(WORK_TEMP) + FAN_OFF_HW_TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_AUTO ,VALVE_OFF);
					    		break;
					    	}
					    	if (usGetRegInput(IN_AIR_TEMP) >= ( usGetReg(WORK_TEMP) + FAN_OFF_HW_TEMP_DELTA))
					    	{
					    		vSetState(FAN_SPEED_OFF,VALVE_OFF);
					    		break;
					    	}
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
					    }
					    if ((usGetRegInput(TYPE) == AW) && ( usGetReg(MODE) == DEV_AUTO))
					    {
					    	if (usGetReg(AIR_TEMP) < (usGetReg(WORK_TEMP) - VALVE_ON_TEMP_DELTA ))
					    	{
					    		vSetState(FAN_SPEED_MID, VALVE_ON);
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) == ( usGetReg(WORK_TEMP) - VALVE_ON_TEMP_DELTA))
					    	{
					    		vSetState( FAN_SPEED_MIN, VALVE_AUTO);
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) <  usGetReg(WORK_TEMP))
					    	{
					    		vSetState( FAN_SPEED_AUTO ,VALVE_AUTO);
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) ==  usGetReg(WORK_TEMP))
					    	{
					    		vSetState( FAN_SPEED_AUTO ,VALVE_OFF );
					    		break;
					    	}
					    	if (usGetReg(AIR_TEMP) < (usGetReg(WORK_TEMP) + SPEED_SWITCH_AW_TEMP_DELTA))
					    	{
					    		vSetState( FAN_SPEED_AUTO ,VALVE_OFF );
					    		break;
					        }
					    	if (usGetReg(AIR_TEMP) >= (usGetReg(WORK_TEMP) + SPEED_SWITCH_AW_TEMP_DELTA))
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
uint8_t mster_control_addres =0;
eMBMasterReqErrCode    errorCode = MB_MRE_NO_ERR;
void vMasterControlFSM()
{
	uint16_t data[4];

	data[0]=usGetReg(MODE) ;
	data[1]=usGetReg(FAN_SPEED_CONFIG);
	data[2]=usGetReg(WORK_TEMP);
	data[3]=usGetReg(AIR_TEMP);

    errorCode = eMBMasterReqWriteMultipleHoldingRegister( 0, 13, 4, &data[0], 0);


	if ( usGetReg( CONTROL_MODE ) )
	{
		mster_control_addres++;
		if ( mster_control_addres > usGetReg( DEVICE_COUNT ) )
		{
			mster_control_addres = 1;
			connection_error = 0;
		}
		errorCode = eMBMasterReqReadHoldingRegister( mster_control_addres, 5, 5, 0 );
		if ( errorCode == MB_MRE_TIMEDOUT )
		{
			connection_error = 1;
		}
	}
}
#endif
