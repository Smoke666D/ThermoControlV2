/*
 * mainFSM.h
 *
 *  Created on: Mar 17, 2023
 *      Author: igor.dymov
 */

#ifndef SRC_MAINFSM_H_
#define SRC_MAINFSM_H_

#include "main.h"
#include "FreeRTOS.h"
#include "event_groups.h"




//#define MASTER_MODE
#define SLAVE_MODE

 void vMainFSM(void *argument);

 void InitSystemEnvet(EventGroupHandle_t event);
#ifdef MASTER_MODE
 void  prvvTIMERExpiredISR();

#endif


#define DIN_READY  0x0001
#define AIN_READY  0x0002
#define MB_READY   0x0004
#define INIT_READY   0x0008
#define PROCESS_DATA   0x0010

#define DEVICE_ADDR_MASK  0x000F
#define DEVICE_ADDR_OFFSET 0x000
#define DEVICE_MODE_MASK  0x0030
#define DEVICE_MODE_OFFSET 4
#define DEVICE_MASTER_CONTROL_MASK 0x80
#define DEVICE_MASTER_CONTROL_OFFSET 7
#define DEVICE_DOOR_MASK  0x100
#define DEVICE_DOOR_OFFSET 8
#define DEVICE_FAN_MASK  0x0300
#define DEVICE_FAN_OFFSET 8
#define DEVICE_TYPE_MASK   0xC00
#define DEVICE_TYPE_OFFSET 10

#define WATER_TEMP_ERROR       0x01
#define AIR_TEMP_ERROR       0x02
#define CONNECTION_ERROR 0x04


#define  DEVICE_DINPUT_START   0
#define  DEVICE_DINPUT         1
#define  DEVICE_COIL_START     (DEVICE_DINPUT_START + DEVICE_DINPUT)
#define  DEVICE_COIL		   4
#define  DEVICE_INPUT_START    ( DEVICE_COIL_START + DEVICE_COIL)
#define  DEVICE_INPUT		   8
#define  DEVICE_HOLDING_START  (DEVICE_INPUT_START + DEVICE_INPUT)
#define  DEVICE_HOLDING        30
#define  DEVICE_HOLDING_FLASG  7
#define REG_COUNT 10





#define FAN_SPEED_1   0x01
#define FAN_SPEED_2   0x02
#define CLOSED  0x01
#define OPEN    0x02
#define REOPEN  0x03

typedef enum
{
 VALVE_ON  = 0x01,
 VALVE_OFF  = 0x00,
 VALVE_AUTO = 0x02,
} VALVE_STATE_t;
typedef enum
 {
    FAN_SPEED_OFF  = 0x00,
    FAN_SPEED_MIN  = 0x01,
    FAN_SPEED_MAX  = 0x03,
	FAN_SPEED_MID  = 0x02,
	FAN_SPEED_AUTO = 0x04,
 } FAN_SPEED_t;

#ifdef SLAVE_MODE
 void rvvTIMERExpiredISR();
#endif

#define DEVICE_TYPE_BN_REG   0x00


#define STANDBY_WATER_ON_TEMP  20
#define STANDBY_WATER_OFF_TEMP 30
#define PREHEAT_OFF_TEMP       20
#define PREHEAT_OFF_TIME       60
#define WATER_FREEZE_TEMP      13
#define VALVE_OFF_TEMP_DELTA 	2
#define VALVE_ON_TEMP_DELTA     2
#define TEMP_DELTA				2
#define DOOR_CLOSE_TIME        120
#define SPEED_SWITCH_AW_TEMP_DELTA 2
#define FAN_OFF_HW_TEMP_DELTA 2
#define SPEED_3_HW_SWITCH_TEMP_DELTA 6
#define SPEED_2_HW_SWITCH_TEMP_DELTA 4
#define SPEED_1_HW_SWITCH_TEMP_DELTA 2
 typedef enum
 {
   TYPE = 0,
   WATER_TEMP = 1,
   IN_AIR_TEMP =2,
   FSM_STATUS = 3,
   WATER_VALVE = 4,
   FAN_SPEED = 5,
   DOOR_STATE_TRIGGER = 6,
   ERROR_STATUS = 7,
   MODE = 0,
   FAN_SPEED_CONFIG = 1,
   WORK_TEMP = 2,
   AIR_TEMP = 3,
#ifdef MASTER_MODE
   CONTROL_MODE = 4,
   DEVICE_COUNT = 5,
   DEVICE_TYPE = 6,
   ERROR_MASTER_STATUS = 7,
#endif
   PWM = 4,
   ADC1_DATA = 5,
   ADC2_DATA = 6,
 } REGS_t;
#define REG_COUNT  10
 typedef enum
 {
 	OFF_MODE,
 	MANUAL_MODE,
 	AUTO_MODE
 } MODE_t;

typedef enum
{
	STANDBAY_STATE,
	WORK_STATE
} MAIN_FSM_STATE_t;


 typedef enum
 {
	 AW = 1,
	 HW = 2,
	 HWC = 3,
	 NONE = 0
 } dev_type_t;

 typedef enum
 {
	 DEV_OFF,
	 DEV_MANUAL,
	 DEV_AUTO

 } dev_mode_t;
typedef enum
{
	INIT_STATE,
	PREHEAT = 1,
	STANDBY = 2,
	WORK = 3,
	TELEMETRY = 4,
	ERROR_STATE = 5,
	INIT_WORK = 6,

} control_flsm_t;

void vUPDATECoils( uint8_t rw);
void vUPDATEDin(uint8_t state );
void vTimer1sInc();
void vDATATask(void *argument);
EventGroupHandle_t xGetSystemControlEvent();
EventGroupHandle_t xGetOSEvent();
EventGroupHandle_t xGetUARTEvent();
uint16_t usGetReg( REGS_t reg_addr);
void vSetReg(REGS_t reg_addr, uint16_t data);
uint16_t usGetRegInput( REGS_t reg_addr);
uint8_t usGetConnection();
void vSetRegInput(REGS_t reg_addr, uint16_t data);
#endif /* SRC_MAINFSM_H_ */
