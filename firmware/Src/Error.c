/**
 * @file 	Error.c
 * @brief 	Set and clear errors.
 *
 * @author 	Tym Zhu
 */
#include "Error.h"

/**
 * @brief Set error bits
 *
 * @param error error
 */
void Error_Set(uint8_t error)
{
	Mode.Error |= error;

}

/**
 * @brief Reset error bits
 *
 * @param error error
 */
void Error_Reset(uint8_t error)
{
	Mode.Error &= (~error);

}
