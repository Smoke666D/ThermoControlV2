/*
 * din_dout.h
 *
 *  Created on: May 11, 2023
 *      Author: i.dymov
 */

#ifndef INC_DIN_DOUT_H_
#define INC_DIN_DOUT_H_

#ifdef SLAVE_MODE
	#define  DIN_CHANNEL  8
	#define  DOUT_CHANNEL  4
#endif
#ifdef MASTER_MODE
	#define  DIN_CHANNEL  12
#endif

#define DEF_H_FRONT 10U
#define DEF_L_FRONT 10U

#define ADC1_CHANNELS      3U
#define ADC_FRAME_SIZE     3U

#define K   ( 3.3 / 0xFFF )

#define RA  10000.0

#define Uvdd 3.3

#define AINCOOF1  (Uvdd*RA- RA)*K // ( ( RA1 + RA3 ) /RA3) * K

typedef enum
{
 DIN_CONFIG_OK = 0,
 DIN_WRONG_CHANNEL_NUMBER  =1
} DIN_FUNCTION_ERROR_t;

typedef enum {
	OUT_CONFIG_NEGATIVE = 0U,
    OUT_CONFIG_POSITIVE = 1U
} DOUT_OUT_TYPE;

typedef enum {
	DIN_CONFIG_NEGATIVE = 0U,
	DIN_CONFIG_POSITIVE = 1U,
	RPM_CONFIG = 2U
} DIN_INPUT_TYPE;

typedef struct {
	uint32_t Pin;
	GPIO_TypeDef * GPIOx;
} PIN_CONFIG;

typedef enum {
		INPUT_1 = 0U,
		INPUT_2 = 1U,
		INPUT_3 = 2U,
		INPUT_4 = 3U,
		INPUT_5 = 4U,
		INPUT_6 = 5U,
		INPUT_7 = 6U,
		INPUT_8 = 7U,
		INPUT_9 = 8U,
		INPUT_10 = 9U,
		INPUT_11 = 10U,
		INPUT_12 = 11U,
		INPUT_13 = 12U,
		INPUT_14 = 13U,
		INPUT_15 = 14U,
		INPUT_16 = 15U,
		INPUT_17 = 16U,
		INPUT_18 = 17U,
		INPUT_19 = 18U,
		INPUT_20 = 19U
} DIN_INPUT_NAME;

typedef enum {
		OUT_1 = 0U,
		OUT_2 = 1U,
		OUT_3 = 2U,
		OUT_4 = 3U
} DOUT_INPUT_NAME;

typedef enum {
		AIN_1 = 0U,
		AIN_2 = 1U,
		AIN_3 = 2U,

} AIN_INPUT_NAME;

typedef struct DoutConfigDef_t
{
	uint8_t       state;
	DOUT_OUT_TYPE eOutConfig;
} DoutConfig_t;

typedef struct DinConfigDef_t
{
uint32_t      ulCounter;
uint32_t 	  ulLowCounter;
uint32_t 	  ulHighCounter;
uint8_t 	  ucValue;
uint8_t 	  ucTempValue;

DIN_INPUT_TYPE eInputType;

} DinConfig_t;
void vADCReady();
 void vDTask(void *argument);
 uint32_t uiGetDinMask();
 void vSetOutState( uint8_t channel, uint8_t state);
 uint8_t vGetOutState(uint8_t channel);
 int16_t iGetTemp( AIN_INPUT_NAME channel );
#endif /* INC_DIN_DOUT_H_ */
