/**
 * @file 	MAX31855.h
 * @brief 	header for MAX31855.c
 *
 * @author 	Tym Zhu
 */
#ifndef MAX31855_H
#define MAX31855_H

#include "main.h"
//#include "Config.h"

/** MAX31855 structure */
typedef struct
{
	uint8_t Fault; 	/**< Fault Code 1:OC 2:GND 4:Vcc */
	float32_t tc; 	/**< thermocouple reading */
	float32_t in; 	/**< internal temperature reading */

} MAX31855_TypeDef;

extern MAX31855_TypeDef TC_Reading;

extern uint8_t MAX31855_Read(MAX31855_TypeDef *Result);

#endif
