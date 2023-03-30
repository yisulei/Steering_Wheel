/**
  **************************** Copyright ******************************
  *
  *                 (C) Copyright 2021, China, IFR»úÆ÷ÈËÊµÑéÊÒ.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_state.h
  * Version		: v1.3
  * Author		: LiuJiawei
  * Date		: 2021-12-06
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_STATE_H_
#define __IFR_STATE_H_



/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */


/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */


typedef enum
{
    CHECK_STATE,		 //×Ô¼ì×´Ì¬
    STOP_STATE,        //Í£Ö¹×´Ì¬
    ERROR_STATE,			 //´íÎó×´Ì¬
    NOMAL_STATE,			 //Õý³£×´Ì¬
    TEST_STATE,
    RESET_STATE,
    CARRY_STATE,
} STATE_TypeDef;

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
extern STATE_TypeDef State;
void State_Machine(void);
static void State_Change(void);
static void State_Task(void);
/* USER CODE END Prototypes */

#endif





