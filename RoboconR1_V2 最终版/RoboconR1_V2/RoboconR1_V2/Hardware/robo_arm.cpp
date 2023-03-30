/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: robo_arm.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "robo_arm.h"

/* Private variables -------------------------------------------------*/
extern YunAndArm_Typedef Robo_YunAndArm;

void Open_AirCylinder(void)
{
	Robo_YunAndArm.Air_Cylinder.High();
}

void Close_AirCylinder(void)
{
	Robo_YunAndArm.Air_Cylinder.Low();
}

void Arm_Motor_Init(void)
{
	//PID 初始化
	Robo_YunAndArm.Motor_Arm.MotorPos_PID_Init			(1.0f, 0.0012f,  	 	 32,	 9000,	6000,		5000,		0);
	Robo_YunAndArm.Motor_Arm.MotorSpeed_PID_Init		(5.0f,	 0.05f,		0.06f, 	 9000,	6000,		300000,	0);
	Robo_YunAndArm.Motor_Wrist.MotorPos_PID_Init		(1.2,				 0,		0.06f,	 9000,	200000,	5000,		0);
	Robo_YunAndArm.Motor_Wrist.MotorSpeed_PID_Init	(4.0f,			 0,  		100,	 9000,	200000,	5000,		0);
	//电机初始化
	Robo_YunAndArm.Motor_Arm.Motor_PosInit();
	Robo_YunAndArm.Motor_Wrist.Motor_PosInit();
}

float Arm_offset_moto(int32_t Tar_Pos) //大臂电机 重力补偿
{
	if(Tar_Pos > -111111) return -3500.0f;
	else return 0;
}

float Wrist_offset_moto(int16_t Tar_Pos) //手腕电机 重力补偿
{
	float tarP_2_moto = 0;
	float tarP_1_moto = 0;
	float offset_value = 0;

	tarP_2_moto = 0.001245f*Tar_Pos*Tar_Pos;
	tarP_1_moto = 18.25f*Tar_Pos;
	offset_value = tarP_2_moto - tarP_1_moto +62050 ;
	
	return offset_value;
}

