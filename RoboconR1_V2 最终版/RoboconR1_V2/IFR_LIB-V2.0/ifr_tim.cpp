/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_tim.cpp
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-19
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_tim.h"

/* Private variables -------------------------------------------------*/
#ifdef HAL_TIM_MODULE_ENABLED
IFR_TIM_ClassDef* TIM_Pointer[15] = {0};

void IFR_TIM_ClassDef::TIM_ITStart(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Start_IT(htim);
}

#if USE_HAL_TIM_REGISTER_CALLBACKS
void IFR_TIM_ClassDef::TIM_ITStart(TIM_HandleTypeDef *htim,void (*TIM_Function)(void))
{
	TIM_Pointer[IFR_TIM_ID_GET(htim)] = this;

	TIM_Overflow_Function = TIM_Function;
	HAL_TIM_RegisterCallback(htim,HAL_TIM_PERIOD_ELAPSED_CB_ID,IFR_TIM_Overflow_Callback);
	HAL_TIM_Base_Start_IT(htim);
}

void IFR_TIM_ClassDef::TaskFunction_Call(void)
{
	if(TIM_Overflow_Function!=NULL)
		(*TIM_Overflow_Function)();
}

void IFR_TIM_Overflow_Callback(TIM_HandleTypeDef *htim)
{
	uint8_t TIM_ID=IFR_TIM_ID_GET(htim);
	IFR_TIM_ClassDef* pTIM_Class = TIM_Pointer[TIM_ID];

	pTIM_Class->TaskFunction_Call();
}

static uint8_t IFR_TIM_ID_GET(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1) 			return 1;
	else if(htim->Instance == TIM2) 	return 2;
	else if(htim->Instance == TIM3) 	return 3;
	else if(htim->Instance == TIM4)  	return 4;
	else if(htim->Instance == TIM5)  	return 5;
	else if(htim->Instance == TIM6) 	return 6;
	else if(htim->Instance == TIM7) 	return 7;
	else if(htim->Instance == TIM8) 	return 8;
	else if(htim->Instance == TIM9)  	return 9;
	else if(htim->Instance == TIM10)  	return 10;
	else if(htim->Instance == TIM11) 	return 11;
	else if(htim->Instance == TIM12)  	return 12;
	else if(htim->Instance == TIM13)  	return 13;
	else if(htim->Instance == TIM14) 	return 14;
	else return 0;
}

#endif
#endif

