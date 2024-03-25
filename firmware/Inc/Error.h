/**
 * @file 	Error.h
 * @brief 	Header for Error.c
 *
 * @author 	Tym Zhu
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ERROR_H
#define ERROR_H

#include "Config.h"

#define	ERROR_INIT				0X40	/**< initialization error */
#define	ERROR_HARDWARE_FAULT	0X20	/**< hardware fault */
#define	ERROR_JOINT_LIMIT       0X10	/**< joint limit */
#define	ERROR_WATCHDOG			0X08	/**< watchdog or external estop signal */
#define	ERROR_ABS_POSI			0X04	/**< absolute position error */
#define	WARNING_OVER_TEMP       0X02	/**< over-temperature warning */
#define	WARNING_COMM			0X01	/**< communication warning */

/** critical errors */
#define	ERROR_CRITICAL	(ERROR_INIT | ERROR_HARDWARE_FAULT)
/** all errors */
#define ERROR_ALL		(ERROR_INIT | ERROR_HARDWARE_FAULT | ERROR_JOINT_LIMIT | ERROR_WATCHDOG)

void Error_Set(uint8_t error);
void Error_Reset(uint8_t error);

#endif 
