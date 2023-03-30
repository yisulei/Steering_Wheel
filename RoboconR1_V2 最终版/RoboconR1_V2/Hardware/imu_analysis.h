/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: imu_analysis.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

#ifndef __IMU_ANALYSIS_H_
#define __IMU_ANALYSIS_H_

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


/* USER CODE BEGIN Private defines */
typedef struct
{
	struct
	{
		float x;
		float y;
		float z;
	}gyro;
	
	struct
	{
	 float Pitch;
	 float Yaw;
	 float Absolute_YAW;
	 float Roll;
	}quaternion;
	
}IMU_Typedef;

extern IMU_Typedef IMU;
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void IMU_Analysis(uint8_t *pData,uint8_t len);
void Absolute_Angle(void);

/* USER CODE END Prototypes */

#endif

