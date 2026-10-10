/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLUEBTN_Pin GPIO_PIN_13
#define BLUEBTN_GPIO_Port GPIOC
#define BLUEBTN_EXTI_IRQn EXTI15_10_IRQn
#define ADXL_DRDY_Pin GPIO_PIN_14
#define ADXL_DRDY_GPIO_Port GPIOE
#define ADXL_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define ADXL_CS_SS_Pin GPIO_PIN_12
#define ADXL_CS_SS_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */
/// Mongoose defines
#define MG_ENABLE_TCPIP                   1 // Enables built-in TCP/IP stack
#define MG_TCPIP_IP                       MG_IPV4(192, 168, 10, 2)
#define MG_TCPIP_MASK                     MG_IPV4(255, 255, 255, 0)
#define MG_TCPIP_GW                       MG_IPV4(0, 0, 0, 0)
#define MG_ENABLE_TCPIP_PRINT_DEBUG_STATS 1 // Enable debug stats log
// #define MG_TCPIP_PHY_ADDR                 0
#define MG_ENABLE_DRIVER_STM32H           1
    // #define MG_ETH_RAM                        __attribute__((section(".eth_ram")))
    //  #define MG_SET_MAC_ADDRESS(mac) \
    //      do \
    //      { \
    //          (mac)[0] = 0x00; \
    //          (mac)[1] = 0x80; \
    //          (mac)[2] = 0xE1; \
    //          (mac)[3] = 0x00; \
    //          (mac)[4] = 0x00; \
    //          (mac)[5] = 0x00; \
    //      } while (0)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
