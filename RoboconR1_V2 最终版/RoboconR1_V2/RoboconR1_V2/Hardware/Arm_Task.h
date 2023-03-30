/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Arm_Task.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

#ifndef __ARM__TASK_H_
#define __ARM__TASK_H_

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
#include "robo_arm.h"
#include "Robo_Task.h"
/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void Arm_Task(State *state);

/* USER CODE END Prototypes */


#endif
