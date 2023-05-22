/*
 * flash_data.h
 *
 *  Created on: Dec 1, 2021
 *      Author: igor.dymov
 */


#include "main.h"


#define FLASH_DATA_ADR 0x800fC00
#define CODE_ADR       		       0x00





#define VALID_CODE   	0x72
#define FLASH_SIZE  	0x08007FFFU
#define APP_ADDRESS    	0x08008000U

void * cgetREGAdr(uint8_t adr);
void vFDSetRegState(uint8_t adr, uint16_t state);
uint16_t vFDGetRegState(uint8_t adr);

