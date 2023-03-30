/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: init.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-03-03
  * Description	: 

  *********************************************************************
  */

#ifndef __INIT_H_
#define __INIT_H_


#endif

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
#include "ifr_lib.h"
#include "tim.h"
#include "usart.h"
#include "can.h"
#include "imu_analysis.h"
#include "vision_analysis.h"
#include "laser_analysis.h"
#include "Robo_Task.h"
#include "Yun_Task.h"
#include "Base_Task.h"
#include "Arm_Task.h"
#include "remote.h"
/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void All_Init(void);
void MotorPID_Init(void);
/* USER CODE END Prototypes */


