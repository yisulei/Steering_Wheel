/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: robo_yun.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

#ifndef __ROBO_YUN_H_
#define __ROBO_YUN_H_

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
#include "imu_analysis.h"
#include "robo_arm.h"
/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
#define Infinite -2147483647
#define Back   	 	10000
#define Mid      			0
#define Front    -10000
 
typedef struct
{
	IFR_GyroControl_Motor		Motor_Pitch;
	IFR_Pos_Motor						Motor_Launch;
	IFR_Pos_Motor 					Motor_Arm;
	IFR_Pos_Motor 					Motor_Wrist;
	IFR_GPIO_ClassDef				Photoelectric_Sensor;
	IFR_GPIO_ClassDef				Air_Cylinder;
	IFR_OPWM_ClassDef 			Friction_PulleyLeft;
	IFR_OPWM_ClassDef 			Friction_PulleyRight;
}YunAndArm_Typedef;

extern YunAndArm_Typedef Robo_YunAndArm;
extern IFR_CAN_ClassDef  CAN_Yun;
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void Open_Frictiongear(uint16_t Frictiongear_Speed);
void Close_Frictiongear(void);
void Yun_Motor_Init(void);
float Yun_pitch_offset_moto(int32_t Pitch_tarP);
/* USER CODE END Prototypes */

#endif
