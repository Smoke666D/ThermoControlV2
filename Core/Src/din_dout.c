/*
 * din_dout.c
 *
 *  Created on: May 11, 2023
 *      Author: i.dymov
 */
#include "main.h"


const uint16_t CalPoint[20][2] = {{0,32742},
								 {5,25451},
								 {10,19936},
								 {15,15731},
								 {17,14337},
								 {18,13693},
								 {20,12500},
								 {21,11948},
								 {22,11423},
								 {23,10925},
								 {25,10000},
								 {26,9570},
								 {27,9162},
								 {28,8773},
								 {29,8403},
								 {30,8051},
								 {35,6523},
								 {40,5315},
								 {45,4355},
							     {50,3593}
};
#define K10	10000
const uint16_t B57164CalPoint[11][2] = {{0,K10*3.5563},
								 {5,K10*2.7119},
								 {10,K10*2.086},
								 {15,K10*1.6204},
								 {20,K10*1.2683},
								 {25,K10},
								 {30,K10*0.7942},
								 {35,K10*0.63268},
								 {40,K10*0.5074},
								 {45,K10*0.41026},
							     {50,K10*0.33363}
};

const uint16_t Resistanse[][2] = {{0,32742},
								 {1,31113},
								 {2,29575},
								 {3,28112},
								 {4,26746},
								 {5,25451},
								 {6,24223},
								 {7,23061},
								 {8,21962},
								 {9,20921},
								 {10,19936},
								 {11,19002},
								 {12,18118},
								 {13,17280},
								 {14,16485},
								 {15,15731},
								 {16,15016},
								 {17,14337},
								 {18,13693},
								 {19,13081},
								 {20,12500},
								 {21,11948},
								 {22,11423},
								 {23,10925},
								 {24,10451},
								 {25,10000},
								 {26,9570},
								 {27,9162},
								 {28,8773},
								 {29,8403},
								 {30,8051},
								 {31,7715},
								 {32,7954},
								 {33,7090},
								 {34,6799},
								 {35,6523},
								 {36,6257},
								 {37,6005},
								 {38,5764},
								 {39,5524},
								 {40,5315},
								 {41,5103},
								 {42,4905},
								 {43,4713},
								 {44,4530},
								 {45,4355},
								 {46,4188},
								 {47,4028},
								 {48,3875},
								 {49,3729},
								 {50,3593}
};

const  PIN_CONFIG xDinPortConfig[DIN_CHANNEL]= {{SW1_Pin,SW1_GPIO_Port},
												{SW2_Pin,SW2_GPIO_Port},
												{SW3_Pin,SW3_GPIO_Port},
												{SW4_Pin,SW4_GPIO_Port},
												{SW5_Pin,SW5_GPIO_Port},
												{SW6_Pin,SW6_GPIO_Port},
												{SW7_Pin,SW7_GPIO_Port},
												{SW8_Pin,SW8_GPIO_Port},
#ifdef MASTER_MODE
												{S1_Pin,S1_GPIO_Port},
												{S2_Pin,S2_GPIO_Port},
												{S3_Pin,S3_GPIO_Port},
												{S4_Pin,S4_GPIO_Port},
#endif
#ifdef SLAVE_MODE
												{DOOR_Pin,DOOR_GPIO_Port}
#endif
};
#ifdef SLAVE_MODE
const PIN_CONFIG xDoutPortConfig[DOUT_CHANNEL] = {{K2_Pin,K2_GPIO_Port},
												  {K4_Pin,K4_GPIO_Port},
												  {K6_Pin,K6_GPIO_Port},
												  {K8_Pin,K8_GPIO_Port}};
#endif
static uint16_t ADC_RAW[ADC1_CHANNELS ];
static uint16_t ADC_OLD_RAW[ADC1_CHANNELS ];
static uint16_t ADC1_DMABuffer[ADC1_CHANNELS *ADC_FRAME_SIZE ];
#ifdef SLAVE_MODE
static DoutConfig_t xDoutConfig[ DOUT_CHANNEL];
#endif
static DinConfig_t xDinConfig[ DIN_CHANNEL];
static EventGroupHandle_t xSystemEventGroupHandle;
static uint8_t DataReadyFlag = 0;
extern ADC_HandleTypeDef hadc1;
#ifdef MASTER_MODE
static uint16_t timeoutR = 0;
static uint16_t timeoutG = 0;
static uint16_t timer = 0;
#endif

 uint16_t vRCFilter( uint16_t input,uint16_t * old_output);
 void vGetAverDataFromRAW(uint16_t * InData, uint16_t *OutData, uint8_t InIndex, uint8_t OutIndex, uint8_t Size, uint16_t BufferSize);
/*
 *
 */
#ifdef SLAVE_MODE
void eOutConfig( uint8_t channel, DOUT_OUT_TYPE type)
{
	if ( channel < DOUT_CHANNEL)
	{
		xDoutConfig[channel].eOutConfig = type;
	}
}
#endif
/*
 *
 */
DIN_FUNCTION_ERROR_t eDinConfig( uint8_t ucCh, DIN_INPUT_TYPE inType, uint32_t ulHFront, uint32_t ulLFront)
{
	DIN_FUNCTION_ERROR_t eRes = DIN_WRONG_CHANNEL_NUMBER ;
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if ( ucCh < DIN_CHANNEL)
	{
		xDinConfig[ucCh].eInputType = inType;
		xDinConfig[ucCh].ucValue 	= (xDinConfig[ucCh].eInputType == DIN_CONFIG_POSITIVE ) ? 0U : 1U;
		GPIO_InitStruct.Pin 		= xDinPortConfig[ucCh].Pin;

		if ( xDinConfig[ucCh].eInputType == RPM_CONFIG )
		{
			xDinConfig[ucCh].eInputType = DIN_CONFIG_POSITIVE;
		}
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(xDinPortConfig[ucCh].GPIOx,&GPIO_InitStruct);
		xDinConfig[ucCh].ulHighCounter = ulHFront;
		xDinConfig[ucCh].ulLowCounter = ulLFront;
		xDinConfig[ucCh].ucTempValue = 0U;
		eRes = DIN_CONFIG_OK;
	}
	return ( eRes );
}


static void vDINInit()
{
	POINT_t d[2];
	eDinConfig( INPUT_1, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_2, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_3, DIN_CONFIG_NEGATIVE, DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_4, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_5, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_6, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_7, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_8, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_9, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
#ifdef MASTER_MODE
	eDinConfig( INPUT_10, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_11, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
	eDinConfig( INPUT_12, DIN_CONFIG_NEGATIVE , DEF_H_FRONT, DEF_L_FRONT );
#endif
#ifdef SLAVE_MODE

	eAinCalDataConfig(AIN_2,20);
	for (int i = 0;i<19;i++)
	{
		d[0].X = CalPoint[i][1];
		d[0].Y = CalPoint[i][0];
		d[1].X = CalPoint[i+1][1];
		d[1].Y = CalPoint[i+1][0];
		eSetAinCalPoint(AIN_2,&d[0],i);
	}
	eAinCalDataConfig(AIN_3,20);
	for (int i = 0;i<19;i++)
	{
		d[0].X = CalPoint[i][1];
		d[0].Y = CalPoint[i][0];
		d[1].X = CalPoint[i+1][1];
		d[1].Y = CalPoint[i+1][0];
		eSetAinCalPoint(AIN_3,&d[0],i);
	}
	eOutConfig( OUT_1, OUT_CONFIG_POSITIVE);
	eOutConfig( OUT_2, OUT_CONFIG_POSITIVE);
	eOutConfig( OUT_3, OUT_CONFIG_POSITIVE);
	eOutConfig( OUT_4, OUT_CONFIG_POSITIVE);
#endif
#ifdef MASTER_MODE
	eAinCalDataConfig(AIN_2,11);
		for (int i = 0;i<11;i++)
		{
			d[0].X = B57164CalPoint[i][1];
			d[0].Y = B57164CalPoint[i][0];
			d[1].X = B57164CalPoint[i+1][1];
			d[1].Y = B57164CalPoint[i+1][0];
			eSetAinCalPoint(AIN_2,&d[0],i);
		}
		eAinCalDataConfig(AIN_3,20);
		for (int i = 0;i<19;i++)
		{
			d[0].X = CalPoint[i][1];
			d[0].Y = CalPoint[i][0];
			d[1].X = CalPoint[i+1][1];
			d[1].Y = CalPoint[i+1][0];
			eSetAinCalPoint(AIN_3,&d[0],i);
		}
#endif
	for (uint8_t i=0; i<ADC1_CHANNELS;i++)
	{
		 ADC_OLD_RAW[i] = 0x00;
	}
}
void vADCReady()
{
	 static portBASE_TYPE xHigherPriorityTaskWoken;
	 xHigherPriorityTaskWoken = pdFALSE;
	 xEventGroupSetBitsFromISR(xSystemEventGroupHandle, AIN_READY, &xHigherPriorityTaskWoken );
	 portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	 return;
}

static GPIO_PinState  ERROR_PIN_STATE = GPIO_PIN_RESET ;
static GPIO_PinState RedLedState;
static GPIO_PinState GreenLedState;
 void vDTask(void *argument)
 {

uint16_t timeout =0;
uint16_t config_temp;
uint8_t error_flag = 0;
uint16_t temp = 0;
uint8_t init_timer = 0;

xSystemEventGroupHandle =  xGetSystemControlEvent();
vDINInit();
	  for(;;)
	  {
#ifdef MASTER_MODE
		  timeoutG = 0;
		  timeoutR = 0;
		  timeout = 0;
		  if ( eGetTypeError() == TYPE_ERROR)
		  {
			  timeoutG = 1;
			  timeout = 250;
		  }
		  if (usGetConnection() == CONNECTION_ERROR_PERESENT)
		  {
			  timeoutR = 1;
			  timeout = 250;
	     }
		  if  ( (usGetRegInput(ERROR_STATUS) &  AIR_TEMP_ERROR ) ||  ( eGetSensError() == SENSOR_ERROR))
		  {
		 	  timeoutG = 1;
			  timeoutR = 1;
		 	  timeout = 500;
		 }
		  if (usGetReg(MODE) == OFF_MODE)
		  {

			    GreenLedState   = (timeoutG) ?  ERROR_PIN_STATE :GPIO_PIN_RESET;
		  		RedLedState   = (timeoutR) ?  ERROR_PIN_STATE :GPIO_PIN_RESET;
		  }

		  else
		  {
			  GreenLedState = (timeoutG) ?  ERROR_PIN_STATE : GPIO_PIN_SET;

		  	  if ((usGetRegInput(TYPE) ==  HWC))
		  	   {
		  		  	RedLedState  = (timeoutR) ?  ERROR_PIN_STATE :  ( (usGetReg(MODE)==2)) ? GPIO_PIN_RESET :GPIO_PIN_SET;
		  		    GreenLedState  = (timeoutG) ?  ERROR_PIN_STATE :  ( (usGetReg(MODE)==2)) ? GPIO_PIN_SET :GPIO_PIN_RESET;
		  	  }
		  	  else
		  	  {
		  		    RedLedState  = (timeoutR) ?  ERROR_PIN_STATE : ((usGetReg(WORK_TEMP) > usGetReg(AIR_TEMP)) ? GPIO_PIN_SET :GPIO_PIN_RESET);
		  	  }
		  }
		  if ( ( timeoutG!=0 ) || ( timeoutR!=0 ))
		  {
		  	  if ( ++timer >= timeout  )
		  	  {
		  	     timer = 0U;
		  	     ERROR_PIN_STATE = ( ERROR_PIN_STATE== GPIO_PIN_RESET) ? GPIO_PIN_SET :GPIO_PIN_RESET;
		  	  }
		  }
		  HAL_GPIO_WritePin( LED_G_GPIO_Port, LED_G_Pin, RedLedState);
		  HAL_GPIO_WritePin( LED_R_GPIO_Port, LED_R_Pin, GreenLedState);
#endif
		    HAL_ADC_Start_DMA(&hadc1,&ADC1_DMABuffer[0], 9);
		  	vTaskDelay(1);
#ifdef SLAVE_MODE
			HAL_GPIO_WritePin( LED_G_GPIO_Port, LED_G_Pin,GPIO_PIN_SET);
#endif
			if (DataReadyFlag == 0)
			{
			   init_timer++;
			   if (init_timer == 50)
			   {
				   DataReadyFlag = 1;
				   xEventGroupSetBits(xSystemEventGroupHandle, DIN_READY );
			   }
			}
#ifdef SLAVE_MODE
			for (uint8_t i= 0U; i < DOUT_CHANNEL; i++)
			{
				HAL_GPIO_WritePin(xDoutPortConfig[i].GPIOx, xDoutPortConfig[i].Pin, xDoutConfig[i].state == 0 ? GPIO_PIN_RESET: GPIO_PIN_SET );
			}
#endif
			for (uint8_t i = 0U; i < DIN_CHANNEL; i++)
					{
						if ( xDinConfig[i].eInputType != RPM_CONFIG )
						{
							uint8_t uсDinState = HAL_GPIO_ReadPin( xDinPortConfig[i].GPIOx, xDinPortConfig[i].Pin);
							if (uсDinState != xDinConfig[i].ucTempValue )
							{
									xDinConfig[i].ulCounter ++ ;
									if (xDinConfig[i].ulCounter > ( (xDinConfig[i].ucTempValue == GPIO_PIN_RESET) ? xDinConfig[i].ulHighCounter : xDinConfig[i].ulLowCounter ) )
									{
												xDinConfig[i].ucValue = uсDinState  ^ ( (~xDinConfig[i].eInputType) & 0x1);
												xDinConfig[i].ucTempValue = uсDinState ;
									}
							}
							else
							{
									xDinConfig[i].ulCounter = 0U;
							}
						}
			}
			xEventGroupWaitBits(xSystemEventGroupHandle,  AIN_READY,  pdFALSE, pdTRUE, portMAX_DELAY );
			HAL_ADC_Stop_DMA(&hadc1);
			vGetAverDataFromRAW(&ADC1_DMABuffer[0],&ADC_RAW[0],0,0,3,3);
#ifdef MASTER_MODE

			vSetReg(DEVICE_TYPE,  (uiGetDinMask() & DEVICE_MODE_MASK)>>DEVICE_MODE_OFFSET );
			vSetReg(DEVICE_COUNT, (uiGetDinMask() & DEVICE_ADDR_MASK)>>DEVICE_ADDR_OFFSET);
			vSetReg(CONTROL_MODE, (uiGetDinMask() & DEVICE_MASTER_CONTROL_MASK)>>DEVICE_MASTER_CONTROL_OFFSET);
			vSetReg(ERROR_MASTER_STATUS, (uiGetDinMask() & DEVICE_MASTER_TEMP_MASK)>>DEVICE_MASTER_TEMP_OFFSET);

			ADC_RAW[0] = vRCFilter(ADC_RAW[0], &ADC_OLD_RAW[0]);
			config_temp = ADC_RAW[0];
			if (config_temp <150)
			{
				config_temp = 150;
			}
			if (config_temp > 3670)
			{
				config_temp = 3670;
			}
			vSetReg(WORK_TEMP , (config_temp-150)/115 +5);
			vSetReg(MODE,(uiGetDinMask() & DEVICE_TYPE_MASK)>>DEVICE_TYPE_OFFSET);
			vSetReg(FAN_SPEED_CONFIG,(uiGetDinMask() & DEVICE_FAN_MASK)>>DEVICE_FAN_OFFSET);
			temp = vAinGetData(AIN_3);
			error_flag = 0;
			if  ((temp>=35000) || (temp <3500))
			{
				if ( usGetReg(ERROR_MASTER_STATUS) )
				{
					vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) | AIR_TEMP_ERROR);
					error_flag = 1;
			    }

				temp = vAinGetData(AIN_2);
				if  ((temp<35000) && (temp >3500))
				{
					vSetReg(AIR_TEMP, (uint16_t)fGetAinCalData(AIN_2,temp));
					if (error_flag ==0)
					{
						vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) & ~AIR_TEMP_ERROR);
					}
				}
				else
				{
					vSetReg(AIR_TEMP, -1);
					vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) | AIR_TEMP_ERROR);
				}

			}
			else
			{
					vSetReg(AIR_TEMP, (uint16_t)fGetAinCalData(AIN_3,temp));
					vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) & ~AIR_TEMP_ERROR);
			}
						//temp1 = vAinGetData(AIN_3);
						//if (temp1>35000)
						//{
						//	vSetRegInput(IN_AIR_TEMP, -1);
						//	vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) | AIR_TEMP_ERROR);
						//}
						//else
						//{
						//	vSetRegInput(IN_AIR_TEMP,(uint16_t)fGetAinCalData(AIN_3,temp1));
						//	vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) & ~AIR_TEMP_ERROR);
						//}
#endif

#ifdef SLAVE_MODE
			vSetReg(ADC1_DATA,vAinGetData(AIN_2));
			vSetReg(ADC2_DATA,vAinGetData(AIN_3) );
			temp = vAinGetData(AIN_2);
			if ((temp>=35000) || (temp <3500))
			{
				vSetRegInput(WATER_TEMP,-1);
				vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) | WATER_TEMP_ERROR);
			}
			else
			{
				vSetRegInput(WATER_TEMP, (uint16_t)fGetAinCalData(AIN_2,temp));
				vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) & ~WATER_TEMP_ERROR);
			}
			temp = vAinGetData(AIN_3);
			if ((temp>=35000) || (temp <3500))
			{
				vSetRegInput(IN_AIR_TEMP, -1);
				vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) | AIR_TEMP_ERROR);
			}
			else
			{
				vSetRegInput(IN_AIR_TEMP,(uint16_t)fGetAinCalData(AIN_3,temp));
				vSetRegInput(ERROR_STATUS,usGetRegInput(ERROR_STATUS) & ~AIR_TEMP_ERROR);
			}

		//	vSetRegInput(WATER_TEMP  , iGetTemp(1));
		//	vSetRegInput(IN_AIR_TEMP , iGetTemp(2));

			vUPDATEDin((uiGetDinMask() & DEVICE_DOOR_MASK)>>DEVICE_DOOR_OFFSET  );
#endif
			vSetRegInput(TYPE, (uiGetDinMask() & DEVICE_MODE_MASK)>>DEVICE_MODE_OFFSET );
	  }

 }

 /*
  * Функция вытаскивает из входного буфера Indata  (размером FrameSize*BufferSize) со смещением InIndex FrameSize отсчетов,
  * счетает среднее арефмитическое и записывает в буффер OutData со смещением OutIndex
  */
  void vGetAverDataFromRAW(uint16_t * InData, uint16_t *OutData, uint8_t InIndex, uint8_t OutIndex, uint8_t Size, uint16_t BufferSize)
 {
 	volatile uint32_t temp;
 	for (uint8_t i=0; i<Size; i++ )
 	{
 		temp = 0;
 		for (uint8_t j=0;j < ADC_FRAME_SIZE; j++ )
 		{
 		  temp += (InData[ InIndex + i + j * BufferSize ]);
 		}
 		OutData[ OutIndex + i ] = temp / ADC_FRAME_SIZE;
 	}
 	return;
 }


#define A 200


 uint16_t vRCFilter( uint16_t input,uint16_t * old_output)
{

	volatile uint32_t new = input;
	volatile uint32_t old = *old_output;
	volatile uint16_t  output =  ( A * old + (256-A)*new )>>8;
	*old_output = output;
	return output;
}

uint16_t vAinGetData(AIN_INPUT_NAME channel)
 {
	 float temp;
	 ADC_RAW[channel] = vRCFilter(ADC_RAW[channel], &ADC_OLD_RAW[channel]);
	 temp =  (float)(ADC_RAW[channel]*RA)/(4095- ADC_RAW[channel]);
	 if (temp > 35000)
		 return 35000;
	 else
	 return (uint16_t)temp;
 }



 int16_t iGetTemp( AIN_INPUT_NAME channel )
 {
	 int i = 0;
	 uint16_t temp = (uint16_t)	vAinGetData(channel );
	 if ((temp < Resistanse[50][1]) || (temp > Resistanse[0][1]))
	 {
		 temp = 0;
	 }
	 else
	 for ( i= 50;i>=0;i--)
	 {
		if (temp<Resistanse[i][1])
		{
			temp = Resistanse[i][0];
			break;
		}
	 }

	 return (uint16_t)	temp;
 }
 /*
  *
 */
 uint8_t ucDinGet( DIN_INPUT_NAME eChNum )
 {
 	return ( (eChNum < DIN_CHANNEL) ? xDinConfig[ eChNum ].ucValue: 0U );
 }
 /*
  *
  */
 uint32_t uiGetDinMask()
 {
 	volatile uint32_t uiMask = 0;
 	for (int8_t i = (DIN_CHANNEL -1);  i > -1 ; i--)
 	{
 		uiMask <<=1;
 		uiMask |= ( xDinConfig[ i ].ucValue & 0x01 );
 	}
 	return ( uiMask );
 }
#ifdef SLAVE_MODE
 /*
  *
  */
 void vSetOutState( uint8_t channel, uint8_t state)
 {
	 xDoutConfig[channel].state = state;
 }
 uint8_t vGetOutState(uint8_t channel)
 {
	 return xDoutConfig[channel].state;
 }
#endif

