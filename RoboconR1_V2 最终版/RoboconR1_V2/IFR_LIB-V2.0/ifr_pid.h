/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_pid.h
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-28
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_PID_H_
#define __IFR_PID_H_

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

/* USER CODE END Includes */

#define abs(x) ((x)>0? (x):(-(x)))

class IFR_PID
{
	public:
		void  PID_Init(float kp,float ki,float kd,float output_max,float error_max,float integral_max,float dead_zone);
		float Positional_PID(float Target_Value,float Actual_Value);
		float Incremental_PID(float Target_Value,float Actual_Value);
		void OutputMax_Set(float output_max);
	private:
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
};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif

