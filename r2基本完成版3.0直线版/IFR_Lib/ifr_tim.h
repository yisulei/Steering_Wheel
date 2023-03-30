/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_tim.h
  * Version		: v1.3
  * Author		: LiJiawei
  * Date		: 2021-12-06
  * Description	: 

  *********************************************************************
  */

#ifndef __IFR_TIM_H_
#define __IFR_TIM_H_




/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

 
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#ifdef HAL_TIM_MODULE_ENABLED

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
#define __IFR_TIM_ID_GET(__HANDLE__) \
((__HANDLE__)->Instance==TIM1 ? 1:\
 (__HANDLE__)->Instance==TIM2 ? 2:\
 (__HANDLE__)->Instance==TIM3 ? 3:\
 (__HANDLE__)->Instance==TIM4 ? 4:\
 (__HANDLE__)->Instance==TIM5 ? 5:\
 (__HANDLE__)->Instance==TIM6 ? 6:\
 (__HANDLE__)->Instance==TIM7 ? 7:\
 (__HANDLE__)->Instance==TIM8 ? 8:\
 (__HANDLE__)->Instance==TIM9 ? 9:\
 (__HANDLE__)->Instance==TIM10?10:\
 (__HANDLE__)->Instance==TIM11?11:\
 (__HANDLE__)->Instance==TIM12?12:\
 (__HANDLE__)->Instance==TIM13?13:\
 (__HANDLE__)->Instance==TIM14?14:0)\

typedef struct
{
	void (*TIM_Overflow_Function)(void);
}TIM_LIST_TypeDef;

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void IFR_TIM_Init(TIM_HandleTypeDef *htim,void(*TIM_Overflow_Function)(void));
void IFR_TIM_Overflow_Callback(TIM_HandleTypeDef *htim);
#endif
/* USER CODE END Prototypes */

#endif
