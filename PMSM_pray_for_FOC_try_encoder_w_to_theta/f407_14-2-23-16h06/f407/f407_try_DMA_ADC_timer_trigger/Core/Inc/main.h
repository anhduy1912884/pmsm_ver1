/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <float.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define CLOCK_FRE 80000000   // clock frequency 80MHz
#define PRESC 8000           // timer5 prescalar
#define HALL_COUNT_TIME 10000 // clock / presc
#define U_DC	30
#define SQRT3	1.73205081
#define SQRT3div2 0.866025
#define PIdiv3	1.04719755
#define pi 3.14159265359
#define pi2 6.28318530718
#define Angle_Ts 0.0002   // 1/5000Hz
#define counterfrequency 16000000
#define PHA1 0 
#define PHA2 1 
#define PHA3 2
#define I_d_ref 0
#define BACK 1
#define FORWARD 0
#define AUTO 1
#define CONTROL 2
#define SUM_ELEC_W_NUM 25
#define gocBanDau  1.57079632679
// 1.0471975512  // 2.09439510239  //  3.14159265359 // 4.18879020479  // 5.23598775598  / pi/2 = 1.57079632679
extern uint16_t period ;
extern uint8_t hall_state  ;
//	alpha_inx += sin_step ;
//	beta_inx += sin_step ;
//	if (alpha_inx > 999) {alpha_inx -=1000;}
//	if (beta_inx > 999) {beta_inx -=1000;}
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
void svpwm(void) ;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
