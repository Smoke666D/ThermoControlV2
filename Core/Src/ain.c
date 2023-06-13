/*
 * ain.c
 *
 *  Created on: 9 мар. 2023 г.
 *      Author: igor.dymov
 */


#include "ain.h"

static AIN_DATA_t xAinData[AIN_NUMBER];
static LIN_COOF   xKoofData[ MAX_COOF_COUNT];
static uint16_t usCurMaxIndex;


void vAINInit()
{
    for (uint8_t i = 0; i < AIN_NUMBER; i++)
    {
        xAinData[i].coof_count = 0U;
        xAinData[i].index = 0U;
    }
    usCurMaxIndex = 0;
    return;
}

/*
 * Функция преобразования данных аналогово канала по клаиборвочым коофициентам
 */
static float fConvertCalData( AIN_NAME_t name, float in_data )
{
    float out_data = 0;
    for (uint16_t i =  xAinData[ name ].index; i < (xAinData[ name ].index + xAinData[ name ].coof_count ); i++ )
    {
        if ( (i ==  (xAinData[ name ].index + xAinData[ name ].coof_count)) || (in_data <= xKoofData[i].data ) )
        {
            out_data = xKoofData[i].k * in_data + xKoofData[i].b;
        }
    }
    return ( out_data );
}
/*
 *
 */
float fGetAinCalData( AIN_NAME_t name, float raw_data)
{
     float out_data = 0U;
      if ( name < AIN_NUMBER )
     {
          if ( xAinData[ name ].coof_count == 0 )
           {
               out_data = raw_data;
           }
           else
           {
               out_data = fConvertCalData( name, raw_data);
           }
      }
      return ( out_data );
}

CAL_ERROR_CODE  eAinCalDataConfig(AIN_NAME_t name, uint8_t cal_point_count )
{
    CAL_ERROR_CODE res = CAL_SUCCESS;
    if (cal_point_count == 1)
    {
        res = CAL_POINT_COUNT_ERROR;
    }
    else
    {
        if (  xAinData[ name ].coof_count == 0 )
        {
            if  ( (cal_point_count +  usCurMaxIndex) > MAX_TOTAL_CAL_POINT )
            {
                res = CAL_MEMORY_FULL;
            }
            else
            {
                xAinData[ name ].coof_count = cal_point_count - 1;
                xAinData[ name ].index = usCurMaxIndex;
                usCurMaxIndex += cal_point_count - 1;
            }
        }
        else
        {
                if ( xAinData[ name ].coof_count != ( cal_point_count- 1))
                {
                    res = CAL_OVERWRITE_ERROR;
                }
        }
    }
    return ( res );
}
/*
 *
 */
CAL_ERROR_CODE  eSetAinCalPoint(AIN_NAME_t name, POINT_t * cal_point, uint16_t PointNumber )
{
	CAL_ERROR_CODE res = CAL_SUCCESS;

	if ( PointNumber < xAinData[ name ].coof_count)
	{
		uint16_t index = PointNumber + xAinData[ name ].index;
		vABLineKoofFinde ( &xKoofData[index].k, &xKoofData[index].b,
									cal_point[0].X, cal_point[ 1 ].X, cal_point[0].Y,cal_point[1].Y);
		xKoofData[index].data = cal_point[ 1 ].X;
	}
	else
	{
		res = CAL_OVERWRITE_ERROR;
	}
	return (res);
}

/*
 *
 */
void vABLineKoofFinde(float * k, float * b,  float x1, float x2, float y1, float y2)
{
		 *k = (float)( y1 - y2 ) /(float) (x1 - x2);
		 *b = (float)(y1*x2 - y2*x1) / (float) (x2-x1);
		 return;
}


