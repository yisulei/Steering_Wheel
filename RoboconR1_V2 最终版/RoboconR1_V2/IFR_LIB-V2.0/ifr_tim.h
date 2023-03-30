/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_tim.h
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-19
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_TIM_H_
#define __IFR_TIM_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
}
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#ifdef HAL_TIM_MODULE_ENABLED
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

class IFR_TIM_ClassDef
{
	public:
		void TIM_ITStart(TIM_HandleTypeDef *htim);
		void TIM_ITStart(TIM_HandleTypeDef *htim,void(*TIM_Function)(void));
		void TaskFunction_Call(void);
	private:
		void (*TIM_Overflow_Function)(void);
};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void IFR_TIM_Overflow_Callback(TIM_HandleTypeDef *htim);
static uint8_t IFR_TIM_ID_GET(TIM_HandleTypeDef *htim);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif
#endif

