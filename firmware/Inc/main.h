/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

//#include "stm32f446xx.h"

#include "arm_math.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_NSS_ENC_Pin GPIO_PIN_4
#define SPI1_NSS_ENC_GPIO_Port GPIOA
#define ADC1_8_Pv_Pin GPIO_PIN_0
#define ADC1_8_Pv_GPIO_Port GPIOB
#define ENC_PRESET_Pin GPIO_PIN_1
#define ENC_PRESET_GPIO_Port GPIOB
#define USART3_CSn_Pin GPIO_PIN_2
#define USART3_CSn_GPIO_Port GPIOB
#define SPI2_NCS_Pin GPIO_PIN_8
#define SPI2_NCS_GPIO_Port GPIOC
#define DRV_ENABLE_Pin GPIO_PIN_9
#define DRV_ENABLE_GPIO_Port GPIOC
#define ENGATE_Pin GPIO_PIN_11
#define ENGATE_GPIO_Port GPIOA
#define Fault_Pin GPIO_PIN_12
#define Fault_GPIO_Port GPIOA
#define SPI3_NSS_DRV_Pin GPIO_PIN_15
#define SPI3_NSS_DRV_GPIO_Port GPIOA
#define SPI3_NSS_TC_Pin GPIO_PIN_2
#define SPI3_NSS_TC_GPIO_Port GPIOD
#define EXT_DI_Pin GPIO_PIN_4
#define EXT_DI_GPIO_Port GPIOB
#define TXEN_Pin GPIO_PIN_5
#define TXEN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


/**
 * @name defines for low-level ports&peripherals
 * @{
 */

/* LEDs (0xFF max)*/
#define LED_R_dim()     TIM5->CCR2 = 32		//low brightness
#define LED_R_bright()  TIM5->CCR2 = 128	//high brightness
#define LED_R_off()		TIM5->CCR2 = 0		//off

#define LED_G_dim()     TIM5->CCR3 = 32		//low brightness
#define LED_G_bright()  TIM5->CCR3 = 128	//high brightness
#define LED_G_off()		TIM5->CCR3 = 0		//off

#define LED_B_dim()     TIM5->CCR4 = 32		//low brightness
#define LED_B_bright()  TIM5->CCR4 = 128	//high brightness
#define LED_B_off()		TIM5->CCR4 = 0		//off


/* IC-MHM Encoder */
#define HSPI_MHM			hspi1
#define HDMA_MHM_TX			hdma_spi1_tx
#define HDMA_MHM_RX			hdma_spi1_rx
#define SPI_MHM_DIS()		SPI1_NSS_ENC_GPIO_Port->BSRR=SPI1_NSS_ENC_Pin //SET
#define SPI_MHM_EN()		SPI1_NSS_ENC_GPIO_Port->BSRR=((uint32_t)SPI1_NSS_ENC_Pin)<<16U //RESET

//PA6     ------> SPI1_MISO
//#define MHM_SLO		(((GPIOA->IDR & GPIO_PIN_6) != 0) ? 1:0)

/* IC-MU Encoder */
#define HSPI_MU				hspi1
#define HDMA_MU_TX			hdma_spi1_tx
#define HDMA_MU_RX			hdma_spi1_rx
#define SPI_MU_DIS()		SPI1_NSS_ENC_GPIO_Port->BSRR=SPI1_NSS_ENC_Pin //SET
#define SPI_MU_EN()			SPI1_NSS_ENC_GPIO_Port->BSRR=((uint32_t)SPI1_NSS_ENC_Pin)<<16U //RESET


/* ADS8353 Current ADC */
#define HSPI_ADS8353			hspi2
#define HDMA_ADS8353_TX			hdma_spi2_tx
#define HDMA_ADS8353_RX			hdma_spi2_rx
#define SPI_ADS8353_DIS()		SPI2_NCS_GPIO_Port->BSRR=SPI2_NCS_Pin //SET
#define SPI_ADS8353_EN()		SPI2_NCS_GPIO_Port->BSRR=((uint32_t)SPI2_NCS_Pin)<<16U //RESET



/* DRV8320 Mosfet driver */
#define HSPI_DRV8320		hspi3
#define DRV8320_EN()		DRV_ENABLE_GPIO_Port->BSRR=DRV_ENABLE_Pin //SET
#define DRV8320_DIS()		DRV_ENABLE_GPIO_Port->BSRR=((uint32_t)DRV_ENABLE_Pin)<<16U //RESET
#define SPI_DRV8320_DIS()	SPI3_NSS_DRV_GPIO_Port->BSRR=SPI3_NSS_DRV_Pin //SET
#define SPI_DRV8320_EN()	SPI3_NSS_DRV_GPIO_Port->BSRR=((uint32_t)SPI3_NSS_DRV_Pin)<<16U //RESET
#define DRV_GATE_EN()		ENGATE_GPIO_Port->BSRR=ENGATE_Pin		//SET
#define DRV_GATE_DIS()		ENGATE_GPIO_Port->BSRR=((uint32_t)ENGATE_Pin)<<16U //RESET


/* MAX31855 Thermal-Couple Reading */
#define HSPI_MAX31855		hspi3
#define SPI_MAX31855_DIS()	SPI3_NSS_TC_GPIO_Port->BSRR=SPI3_NSS_TC_Pin //SET
#define SPI_MAX31855_EN()	SPI3_NSS_TC_GPIO_Port->BSRR=((uint32_t)SPI3_NSS_TC_Pin)<<16U //RESET


/* TMP112 I2C temperature reading for mosfet */
#define HI2C_TMP112		hi2c1


/*RS485 */
#define RS485_TXEN()	TXEN_GPIO_Port->BSRR=TXEN_Pin //SET
#define RS485_RXEN()	TXEN_GPIO_Port->BSRR=((uint32_t)TXEN_Pin)<<16U //RESET
/* RS485 usart define */
#define USART_RS485		USART1
#define HUART_RS485		huart1


/*serial port for debug*/
#define USART_DEBUG		USART6
#define HUART_DEBUG		huart6


/* External encoder*/
#define USART_EXT_ENC			USART3
#define HUSART_EXT_ENC			husart3

#define SSI_EXT_ENC_DIS()		USART3_CSn_GPIO_Port->BSRR=USART3_CSn_Pin //SET
#define SSI_EXT_ENC_EN()		USART3_CSn_GPIO_Port->BSRR=((uint32_t)USART3_CSn_Pin)<<16U //RESET


/*** External estop define ***/
/* 0 = unestop   1 = estop */
#define EXT_ESTOP		(((EXT_DI_GPIO_Port->IDR & EXT_DI_Pin) != 0) ? 1:0)	



/*Debug */
//#define Debug_Avg_Size 1000	//not in use
//#define Debug_Log_Size 40000	//MAX=174762


#define HTIM_SLOW_LOOP		 htim6

/**
 * @}
 */


/***** variables *****/

extern volatile uint32_t idle_ct;
extern volatile uint8_t idle_refresh;



/***********************************************/

extern ADC_HandleTypeDef hadc1;

extern CRC_HandleTypeDef hcrc;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;

extern TIM_HandleTypeDef htim1;	//PWMs
extern TIM_HandleTypeDef htim5;	//LEDs
extern TIM_HandleTypeDef htim6;	//Slow loop 5Hz
extern TIM_HandleTypeDef htim7; //Mid loop  4000Hz

extern UART_HandleTypeDef huart1;
extern USART_HandleTypeDef husart3;
extern UART_HandleTypeDef huart6;




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
