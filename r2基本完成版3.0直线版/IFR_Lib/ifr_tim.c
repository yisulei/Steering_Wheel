/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_tim.c
  * Version		: v1.3
  * Author		: LiJiawei
  * Date		: 2021-12-06
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_tim.h"

/* Private variables -------------------------------------------------*/

#if USE_HAL_TIM_REGISTER_CALLBACKS 
#ifdef HAL_TIM_MODULE_ENABLED
TIM_LIST_TypeDef TIM_List[15] = {0};


/**
  * 函数名称：IFR_TIM_Init
  * 函数功能：使能定时器中断，并指定中断时要执行的函数
  * 入口参数：定时器编号、定时器中断时要执行的函数
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-12-06
  */
void IFR_TIM_Init(TIM_HandleTypeDef *htim,void(*TIM_Overflow_Function)(void))
{
	uint8_t TIM_ID=__IFR_TIM_ID_GET(htim);
	TIM_List[TIM_ID].TIM_Overflow_Function=TIM_Overflow_Function;
	
	HAL_TIM_RegisterCallback(htim,HAL_TIM_PERIOD_ELAPSED_CB_ID,IFR_TIM_Overflow_Callback);
	HAL_TIM_Base_Start_IT(htim);
}

/**
  * 函数名称：IFR_TIM_Init
  * 函数功能：定时器接收中断回调函数
  * 入口参数：定时器编号
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-10-27
  */
void IFR_TIM_Overflow_Callback(TIM_HandleTypeDef *htim)
{
	uint8_t TIM_ID=__IFR_TIM_ID_GET(htim);
	
	if(TIM_List[TIM_ID].TIM_Overflow_Function!=NULL)
		TIM_List[TIM_ID].TIM_Overflow_Function();
}

#endif


#endif
