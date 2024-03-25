/**
 * @file 	Multiturn_Enc.h
 * @brief 	Header for Multiturn_Enc.c
 *
 * @author 	Tym Zhu
 */
#ifndef MULTITURN_ENC_H
#define MULTITURN_ENC_H

//#include "main.h"
//#include "Config.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

/**
 * @name Multi-turn macros
 * @{
 */
#define ENC_MAX_RETRY	100

#define MT_CANDIDATES		((int32_t)(MAX_TURN*REDUCTION_RATIO))
#define MT_CANDIDATE_MIN	((int32_t)(-MT_CANDIDATES/2))

//#define INIT_MT_CANDIDATES		((int32_t)(ceilf(0.5f*REDUCTION_RATIO))*2+2)
//#define INIT_MT_CANDIDATE_MIN	(-INIT_MT_CANDIDATES/2)
#define INIT_MT_TOLERANCE		(0.25f*2.0f*PI/REDUCTION_RATIO)


//static const int8_t Turn_301[]={	16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
//								-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,
//								-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1};
//static const int8_t Turn_230[]={	0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
//								16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
//								-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17};

/**
 * @}
 */

/** encoder calibration */
typedef struct
{
	uint8_t Valid;
	uint32_t Retry_CT;
	uint16_t RGB_SPIRCV[2];

	float32_t MIN_1st, MIN_2nd, ExtMIN_1st;
	int32_t MIN_Nturn_1st, MIN_Nturn_2nd, MIN_Nturn_1st_Ext;
} Encoder_Cal_TypeDef;

/** multi-turn data structure */
typedef struct
{
	uint8_t State; /* 0:Idle  1:Primed  2:Calculate*/
	uint8_t Error;
	float32_t Best_Theta_Candidate_delta;
	int32_t Best_MT, MT_Offset;

	float32_t Exp_Theta, Tolerance;

} MLT_MT_TypeDef;

extern MLT_MT_TypeDef MLT_MT;

extern uint8_t MLT_Find_MT(int32_t ST, int32_t MT);

extern uint8_t MLT_Init_MT(int32_t ST, int32_t MT, int32_t Ext_Theta);

#endif

