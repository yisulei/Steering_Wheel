/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: robo_arm.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

#ifndef __ROBO_ARM_H_
#define __ROBO_ARM_H_

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
#include "robo_yun.h"
/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void Arm_Motor_Init(void);
void Open_AirCylinder(void);
void Close_AirCylinder(void);
float Arm_offset_moto(int32_t Tar_Pos); //大臂电机 重力补偿
float Wrist_offset_moto(uint16_t Tar_Pos); //手腕电机 重力补偿
/* USER CODE END Prototypes */

#endif

