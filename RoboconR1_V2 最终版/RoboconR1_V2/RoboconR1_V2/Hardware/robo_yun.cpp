/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: robo_yun.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "robo_yun.h"

/* Private variables -------------------------------------------------*/
IFR_CAN_ClassDef 			CAN_Yun;
YunAndArm_Typedef Robo_YunAndArm ={0x201,0x202,IFR_Pos_Motor(Arm_offset_moto,0x204),0x203,\
																	IFR_GPIO_ClassDef(GPIOA,Photoelectric_Sensor_Pin),
																	IFR_GPIO_ClassDef(GPIOA,Photoelectric_Sensor_Pin)};
extern IMU_Typedef IMU;

void Open_Frictiongear(uint16_t Frictiongear_Speed)
{
	Robo_YunAndArm.Friction_PulleyLeft.OPWM_SetCompare(Frictiongear_Speed);
	Robo_YunAndArm.Friction_PulleyRight.OPWM_SetCompare(Frictiongear_Speed);
}

void Close_Frictiongear(void)
{
	Robo_YunAndArm.Friction_PulleyLeft.OPWM_SetCompare(1000);
	Robo_YunAndArm.Friction_PulleyRight.OPWM_SetCompare(1000);
}

void Yun_Motor_Init(void)
{
	//PID初始化
	Robo_YunAndArm.Motor_Pitch.MotorPos_PID_Init		(1.0f, 0.0012f,  		 32,	  500,	6000,		5000,		0);
	Robo_YunAndArm.Motor_Pitch.MotorSpeed_PID_Init	(5.0f,	 0.05f,		0.06f,	 9000,	6000,		300000,	0);
	Robo_YunAndArm.Motor_Launch.MotorPos_PID_Init		(1.2f,			 0,		0.06f,	 9000,	200000,	5000,		0);
	Robo_YunAndArm.Motor_Launch.MotorSpeed_PID_Init	(4.0f,			 0,  		30,	 9000,	200000,	5000,		0);
	//电机标定
//	while(Robo_YunAndArm.Photoelectric_Sensor.Read() != GPIO_PIN_SET) 
//	{
//		Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Infinite);
//		CAN_Yun.CAN_TransmitForMotor();
//	}
	Robo_YunAndArm.Motor_Launch.output = 0;
//	CAN_Yun.CAN_TransmitForMotor();
	HAL_Delay(100);
	Robo_YunAndArm.Motor_Launch.Motor_PosInit();	
	Robo_YunAndArm.Motor_Pitch.Motor_PosInit();
}

void Pitch_Motor_Init(void)//陀螺仪标定
{
	//可能暂时使用机械标定
}

float Yun_pitch_offset_moto(uint16_t Pitch_tarP) //云台pitch电机 重力补偿
{
	float tarP_2_moto = 0;
	float tarP_1_moto = 0;
	float offset_value = 0;

	tarP_2_moto = 0.001245f*Pitch_tarP*Pitch_tarP;
	tarP_1_moto = 18.25f*Pitch_tarP;
	offset_value = tarP_2_moto - tarP_1_moto +62050 ;
	
	return offset_value;
}

