#ifndef    USER_APP
#define USER_APP
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbutils.h"
#include "mainFSM.h"
/* -----------------------Slave Defines -------------------------------------*/
#define S_DISCRETE_INPUT_START        DEVICE_DINPUT_START
#define S_DISCRETE_INPUT_NDISCRETES   DEVICE_DINPUT
#define S_COIL_START                  DEVICE_COIL_START
#define S_COIL_NCOILS                 DEVICE_COIL
#define S_REG_INPUT_START             DEVICE_INPUT_START
#define S_REG_INPUT_NREGS             DEVICE_INPUT
#define S_REG_HOLDING_START           DEVICE_HOLDING_START
#define S_REG_HOLDING_NREGS           DEVICE_HOLDING
/* salve mode: holding register's all address */
#define          S_HD_RESERVE                     0
#define          S_HD_CPU_USAGE_MAJOR             1
#define          S_HD_CPU_USAGE_MINOR             2
/* salve mode: input register's all address */
#define          S_IN_RESERVE                     0
/* salve mode: coil's all address */
#define          S_CO_RESERVE                     0
/* salve mode: discrete's all address */
#define          S_DI_RESERVE                     0

/* -----------------------Master Defines -------------------------------------*/
#define M_DISCRETE_INPUT_START        0
#define M_DISCRETE_INPUT_NDISCRETES   16
#define M_COIL_START                  0
#define M_COIL_NCOILS                 64
#define M_REG_INPUT_START             0
#define M_REG_INPUT_NREGS             100
#define M_REG_HOLDING_START           0
#define M_REG_HOLDING_NREGS           100
/* master mode: holding register's all address */
#define          M_HD_RESERVE                     0
/* master mode: input register's all address */
#define          M_IN_RESERVE                     0
/* master mode: coil's all address */
#define          M_CO_RESERVE                     0
/* master mode: discrete's all address */
#define          M_DI_RESERVE                     0

void vMVRegInputWrite( uint16_t usData, uint16_t usAddress);
uint16_t vMVRegInputRead(  uint16_t usAddress);

#endif
