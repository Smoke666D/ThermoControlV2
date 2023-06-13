/*
 * ain.h
 *
 *  Created on: 9 мар. 2023 г.
 *      Author: igor.dymov
 */

#ifndef AIN_H_
#define AIN_H_

#include "main.h"



#define MAX_CAL_POINT  20
#define MAX_TOTAL_CAL_POINT  ( AIN_NUMBER * MAX_CAL_POINT  ) //Суммароне количество калибровочных точек
#define MAX_COOF_COUNT       ( AIN_NUMBER * ( MAX_CAL_POINT -1 ) )
typedef enum {
  AIN1 = 0,
  AIN2 = 1,
  AIN3 = 2,
} AIN_NAME_t;

/*Коофиценты для кривых */
typedef struct
{
    float data;
    float k;
    float b;
}   LIN_COOF;

typedef struct
{
    uint16_t KOOF;
    float Data;
} KAL_DATA;


typedef struct
{
    float Y;
    float X;
} POINT_t;

typedef  struct
{
    uint8_t     coof_count;
    uint8_t     index;

} AIN_DATA_t;

typedef enum {
 CAL_SUCCESS= 0,
 CAL_MEMORY_FULL = 1,
 CAL_OVERWRITE_ERROR = 2,
 CAL_POINT_COUNT_ERROR = 3
} CAL_ERROR_CODE;

CAL_ERROR_CODE  eAinCalDataConfig(AIN_NAME_t name, uint8_t cal_point_count );
CAL_ERROR_CODE  eSetAinCalPoint(AIN_NAME_t name, POINT_t * cal_point, uint16_t PointNumber );
CAL_ERROR_CODE  eSetAinCal(AIN_NAME_t name, POINT_t * cal_point, uint8_t cal_point_count);
void vABLineKoofFinde(float * k, float * b,  float x1, float x2, float y1, float y2);
float fGetAinCalData( AIN_NAME_t name, float raw_data);
void vAINInit();

#endif /* AIN_H_ */
