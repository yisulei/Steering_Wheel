/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_gpio.h
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-18
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_GPIO_H_
#define __IFR_GPIO_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {

#ifdef __cplusplus
}
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

class IFR_GPIO_ClassDef
{
	public:
		IFR_GPIO_ClassDef(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
		void High(void);
		void Low(void);
		void Turn(void);
		GPIO_PinState Read(void);
	private:
		GPIO_TypeDef* GPIOX;
		uint16_t GPIO_PIN;
		GPIO_PinState PinState;
};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif
#endif

