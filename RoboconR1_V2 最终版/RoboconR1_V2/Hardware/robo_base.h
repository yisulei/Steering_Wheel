/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: robo_base.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

#ifndef __ROBO_BASE_H_
#define __ROBO_BASE_H_

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

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
#define F 0
#define LB 1
#define RB 2
 
typedef struct
{
	IFR_Speed_Motor 				Motor_Move[3] ;
	IFR_Pos_Motor						Motor_Turn[3] ;
	
	float Vel_Y;
	float Vel_X;
	float Vel_W;
}Base_Typedef;

extern IFR_CAN_ClassDef		CAN_Base;
extern Base_Typedef Robo_Base;
extern IFR_PID BaseFollow_PID;
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void Base_Motor_Init(void);
void Base_Control(float Vel_Y,float Vel_X,float Vel_W);
static void ThreeSteering_MoveIK(float Vel_Y,float Vel_X,float Vel_W);
static void MotorTarget_Set();
void duolun_biaoding(void);
static float Rad_Turn_MotorAngle(float angle);
float Vel_Convert(float Vel);
/* USER CODE END Prototypes */


#endif

