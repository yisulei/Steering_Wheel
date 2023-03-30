/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_pid.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2021-11-14
  * Description	: 

  *********************************************************************
  */

#ifndef __IFR_PID_H_
#define __IFR_PID_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef struct 
{
	float Kp;           
	float Ki;       
	float Kd;
	float Error;
	float Error_Max;
	float Integral;  
	float Integral_Max;
	float Diffrential;   
	float Dead_Zone;
	float Last_Error;
	float Last_Last_Error;
	float Output;
	float Output_Max;
}PID_TypeDef;
/* USER CODE END Private defines */
#define abs(x) ((x)>0? (x):(-(x)))
/* USER CODE BEGIN Prototypes */
void IFR_PID_Param_Init(PID_TypeDef *Pid,float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone);

int IFR_Positional_PID(PID_TypeDef *Pid,float Target_Value,float Actual_Value);

int IFR_Incremental_PID(PID_TypeDef *Pid,float Target_Value,float Actual_Value);
/* USER CODE END Prototypes */

#endif
