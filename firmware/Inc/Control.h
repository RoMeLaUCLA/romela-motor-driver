/**
 * @file 	Control.h
 * @brief 	Header file Control.c
 *
 * @author 	Tym Zhu
 */
#ifndef CONTROL_H
#define CONTROL_H

#include "Config.h"

extern void Ctrl_Init();

extern void ADC_Routine();
extern void Mid_Ctrl_Routine();
extern void Slow_Loop_Routine();

/** cpu percentage structure */
typedef struct
{
	uint32_t CPU_Percent;
	uint32_t ADC_Freq;
	uint32_t MidCtrl_Freq;
} CPUPercent_TypeDef;



//extern arm_biquad_casd_df1_inst_f32 IIR_Vbus;
//extern float32_t IIR_Vbus_State[4];
//extern arm_biquad_casd_df1_inst_f32 IIR_Posi_ref;
//extern float32_t IIR_Posi_ref_State[4];
//extern arm_biquad_casd_df1_inst_f32 IIR_TempTC;
//extern float32_t IIR_TempTC_State[4];

#endif
