/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_gpio.cpp
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-18
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_gpio.h"

/* Private variables -------------------------------------------------*/

IFR_GPIO_ClassDef::IFR_GPIO_ClassDef(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	PinState  = GPIO_PIN_RESET;
	GPIOX = GPIOx;
	GPIO_PIN = GPIO_Pin;
}

void IFR_GPIO_ClassDef::High(void)
{
	PinState  = GPIO_PIN_SET;
	HAL_GPIO_WritePin(GPIOX,GPIO_PIN,GPIO_PIN_SET);
}

void IFR_GPIO_ClassDef::Low(void)
{
	PinState  = GPIO_PIN_RESET;
	HAL_GPIO_WritePin(GPIOX,GPIO_PIN,GPIO_PIN_RESET);
}

void IFR_GPIO_ClassDef::Turn(void)
{
	HAL_GPIO_TogglePin(GPIOX,GPIO_PIN);
	PinState  =  HAL_GPIO_ReadPin(GPIOX,GPIO_PIN);
}

GPIO_PinState IFR_GPIO_ClassDef::Read(void)
{
	PinState = HAL_GPIO_ReadPin(GPIOX,GPIO_PIN);
	return PinState;
}

