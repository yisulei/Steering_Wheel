/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFRï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Êµï¿½ï¿½ï¿½ï¿½.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: imu_analyse.h
  * Version		: v1.0
  * Author		: ÒæËÕÀÙ 
  * Date		: 2021-12-20
  * Description	: 

  *********************************************************************
  */

#ifndef __IMU_ANALYSE_H_
#define __IMU_ANALYSE_H_




/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Chassis_behavior.h"


typedef struct
{
	float Angle_Pitch;
	float Angle_Roll;
	float Angle_Yaw;
	float Speed_Pitch;
	float Speed_Roll;
	float Speed_Yaw;
	float Last_YAW;
}IMU_DATA;

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

void IMU_Analysis(uint8_t *pData,uint8_t len);
void Absolute_Angle(void);
#endif


