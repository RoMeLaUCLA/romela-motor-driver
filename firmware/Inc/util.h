/**
 * @file 	util.h
 * @brief 	header for util.c
 *
 * @author 	Tym Zhu
 */

#include "stm32f4xx_hal.h"
#include "arm_math.h"

extern void DelayMicros(uint32_t micros);


extern int64_t WrapRtrTheta64(int64_t x);
extern float32_t WrapThetaF(float32_t x);
extern int32_t WrapRtrThetaU32(int32_t x);
