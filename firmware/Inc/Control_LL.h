/**
 * @file 	Control_LL.h
 * @brief 	Header for Control_LL.c
 *
 * @author 	Tym Zhu
 */

#ifndef CONTROL_LL_H
#define CONTROL_LL_H

//#include "main.h"
#include "Config.h"


extern void Motor_Enable();
extern void Motor_Disable();

extern void Motor_Calc_Ialpha_Ibeta(Motor_TypeDef *MT);
extern void Motor_CalculateOutput(Motor_TypeDef *MT);

extern void AFC_Init(AFC_TypeDef *afc, float32_t harm, float32_t k,
		float32_t max);
extern float32_t AFC_Calc(AFC_TypeDef *afc, float32_t theta, float32_t err);

extern uint32_t Get_Tick_Diff();

extern void arm_biquad_cascade_df1_f32_1Stage1Sample();

extern void Motor_Theta_ref_S_Curve(float32_t *cmd, float32_t *ref,
		float32_t *vel, float32_t *acc);

extern void Save_Config();
extern void Trig_Slow_Loop();


#endif

