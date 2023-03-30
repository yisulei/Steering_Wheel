/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China,IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_pwm.cpp
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-22
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_pwm.h"

/* Private variables -------------------------------------------------*/
#ifdef HAL_TIM_MODULE_ENABLED

void IFR_OPWM_ClassDef::OPWM_Init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	__htim = htim; __Channel = Channel;
	HAL_TIM_PWM_Start(htim,Channel);
}

void IFR_OPWM_ClassDef::OPWM_SetCompare(uint16_t __Compare)
{
	__HAL_TIM_SET_COMPARE(__htim,__Channel,__Compare);
}
#if USE_HAL_TIM_REGISTER_CALLBACKS
IFR_IPWM_ClassDef *IPWM_Pointer[6][5];
void IFR_IPWM_ClassDef::IPWM_Init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	IPWM_Pointer[IFR_PWMTIM_ID_GET(htim)][IFR_PWMChannel_ID_GET(htim)] = this;

	__htim = htim; __Channel = Channel;

	__HAL_TIM_SET_PRESCALER(__htim,84-1);
	__HAL_TIM_SET_AUTORELOAD(__htim,65535);
	HAL_TIM_RegisterCallback(htim,HAL_TIM_PERIOD_ELAPSED_CB_ID,IFR_PWM_TIM_Overflow_Callback);
	HAL_TIM_RegisterCallback(htim,HAL_TIM_IC_CAPTURE_CB_ID,IFR_TIM_PWMCapture_Callback);
	HAL_TIM_Base_Start_IT(htim);
	HAL_TIM_IC_Start_IT(htim,Channel);
}

void IFR_PWM_TIM_Overflow_Callback(TIM_HandleTypeDef *htim)
{
	IFR_IPWM_ClassDef *pIPWM_Class = IPWM_Pointer[IFR_PWMTIM_ID_GET(htim)][IFR_PWMChannel_ID_GET(htim)];
	pIPWM_Class->IPWM_StartAndCalculate();
}

void IFR_TIM_PWMCapture_Callback(TIM_HandleTypeDef *htim)
{
	IFR_IPWM_ClassDef *pIPWM_Class = IPWM_Pointer[IFR_PWMTIM_ID_GET(htim)][IFR_PWMChannel_ID_GET(htim)];
	pIPWM_Class->IPWM_Capture();
}

void IFR_IPWM_ClassDef::IPWM_StartAndCalculate(void)
{
	switch (Capture_Cnt)
	{
		case 0:
			Capture_Cnt++;
			__HAL_TIM_SET_CAPTUREPOLARITY(__htim, __Channel, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(__htim, __Channel);
			break;
		case 4:
			PWM = (float)High_Time/(float)PWM_Period;

			Number_Overflow = 0;
			Capture_Cnt = 0;
			__HAL_TIM_SET_COUNTER(__htim,0);
			break;
		default:
			Number_Overflow++;
			break;
	}
}

void IFR_IPWM_ClassDef::IPWM_Capture(void)
{
	switch(Capture_Cnt)
	{
		case 1:
		{
			Capture_Buf[0] = HAL_TIM_ReadCapturedValue(__htim,__Channel);
			__HAL_TIM_SET_CAPTUREPOLARITY(__htim,__Channel,TIM_ICPOLARITY_FALLING);
			Capture_Cnt++;
			break;
		}
		case 2:
		{
			Capture_Buf[1] = HAL_TIM_ReadCapturedValue(__htim,__Channel);
			__HAL_TIM_SET_CAPTUREPOLARITY(__htim, __Channel, TIM_INPUTCHANNELPOLARITY_RISING);
			High_Time =	Number_Overflow * 65536 + Capture_Buf[1] - Capture_Buf[0];
			Capture_Cnt++;
			break;
		}
		case 3:
		{
			Capture_Buf[2] = HAL_TIM_ReadCapturedValue(__htim,__Channel);
			HAL_TIM_IC_Stop_IT(__htim,__Channel);
			PWM_Period = Number_Overflow * 65536 + Capture_Buf[2]  - Capture_Buf[0];
			Low_Time = PWM_Period - High_Time;
			Capture_Cnt++;
			break;
		}
	}
}

int32_t IFR_IPWM_ClassDef::HighTime_Get(void)
{
	return High_Time;
}

int32_t IFR_IPWM_ClassDef::LowTime_Get(void)
{
	return Low_Time;
}

int32_t IFR_IPWM_ClassDef::Period_Get(void)
{
	return PWM_Period;
}

float 	IFR_IPWM_ClassDef::PWM_Get(void)
{
	return PWM;
}

static uint8_t IFR_PWMTIM_ID_GET(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM8) 	return 1;
	else if(htim->Instance == TIM2) 	return 2;
	else if(htim->Instance == TIM3) 	return 3;
	else if(htim->Instance == TIM4)  	return 4;
	else if(htim->Instance == TIM5)  	return 5;
	else return 0;
}

static uint8_t IFR_PWMChannel_ID_GET(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) 	return 1;
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) 	return 2;
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  	return 3;
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  	return 4;
	else return 0;
}
#endif
#endif

