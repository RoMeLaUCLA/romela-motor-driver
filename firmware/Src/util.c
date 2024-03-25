/**
 * @file 	util.c
 * @brief 	Useful utility functions.
 *
 * @author 	Tym Zhu
 */
#include "util.h"

#include "Config.h"

//#pragma optimize=medium
#pragma GCC push_options
#pragma GCC optimize ("O0")
/**
 * @brief Delay in micro seconds
 *
 * @param micros
 */
void DelayMicros(uint32_t micros)
{
	/* Multiply micros with multipler */
	/* Substract 10 TODO: test */
	micros = micros * 36 - 10;	//for hclk 180MHz
	/* 5 cycles for one loop */
	while (micros--){};
}

#pragma GCC pop_options


/**
 * @brief wrap int64 rotor theta
 *
 * @param x input
 * @return  wrapped value
 */
inline int64_t WrapRtrTheta64(int64_t x)
{
	x = (x - RTR_THETA_MIN) % RTR_THETA_FULL_RES;
	if (x < 0)
		x += RTR_THETA_FULL_RES;
	return (x + RTR_THETA_MIN);
}

/**
 * @brief wrap float32 rotor theta
 *
 * @param x input
 * @return  wrapped value
 */
inline float32_t WrapThetaF(float32_t x)
{
	x = fmodf((x - THETA_MIN_F), THETA_FULL_RES_F);
	if (x < 0.0f)
		x += THETA_FULL_RES_F;
	return (x + THETA_MIN_F);
}

/**
 * @brief wrap int32 rotor theta
 *
 * @param x input
 * @return  wrapped value
 */
inline int32_t WrapRtrThetaU32(int32_t x)
{
	x = x % RTR_ENC_RES;
	if (x < 0)
		x += RTR_ENC_RES;
	return (x);
}
