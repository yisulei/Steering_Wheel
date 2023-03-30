/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Robo_Task.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

#ifndef __ROBO__TASK_H_
#define __ROBO__TASK_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "ifr_remote.h"
/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
typedef enum
{
  ERROR_STATE,//错误状态
  START_STATE,//开始状态
  CHECK_STATE,//自检状态
  MOVE_STATE, //移动状态
  GRAB_STATR, //抓取状态
  SHOOT_STATE,//自瞄状态
  STOP_STATE, //停止状态
  SMART_STATE, //半自动状态
	MANUAL_STATE,//手动状态
	MANUALSHOOT_STATE,//手动射击模式
}State;

extern State state;
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void Change_State(void);

/* USER CODE END Prototypes */


#endif
