/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Yun_Task.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "Yun_Task.h"

/* Private variables -------------------------------------------------*/
#define Pitch_Max 	 -38000
#define Pitch_Min 				0
#define Pitch_Mid 	 			0
//(Pitch_Max-Pitch_Min)/4.0f

#define Clip_Max 	 	 -5000000
#define Clip_Min 					  0
#define Clip_Mid 	 	 			  0

int32_t Pitch_Tar = Pitch_Mid;
extern Vision_Typedef vision_data;

void Yun_Task(State *state)
{
	switch(*state)
	{
    case ERROR_STATE:
			Robo_YunAndArm.Motor_Pitch.output = 0;
			Close_Frictiongear();
			break;
    case STOP_STATE:
			Robo_YunAndArm.Motor_Pitch.Motor_Pos_Set(Pitch_Mid);
			Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Mid);
			Close_Frictiongear();
			break;
    case MOVE_STATE:
			Pitch_Tar -=(DJI_Remote.Chy_Right-1024)*20.0f/660.0f;
			if(Pitch_Tar < Pitch_Max)	Pitch_Tar = Pitch_Max;
			if(Pitch_Tar > Pitch_Min)	Pitch_Tar = Pitch_Min;	
			Robo_YunAndArm.Motor_Pitch.Motor_Pos_Set(Pitch_Tar);
			Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Mid);
			Close_Frictiongear();		
			break;
    case GRAB_STATR:
			Robo_YunAndArm.Motor_Pitch.Motor_Pos_Set(Pitch_Mid);
			Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Min);
			Close_Frictiongear();
			break;
    case SHOOT_STATE:
			Open_Frictiongear(1800);
			Pitch_Tar -= vision_data.Pitch_Error/15000.0f;
			if(Pitch_Tar < Pitch_Max)	Pitch_Tar = Pitch_Max;
			if(Pitch_Tar > Pitch_Min)	Pitch_Tar = Pitch_Min;	
			Robo_YunAndArm.Motor_Pitch.Motor_Pos_Set(Pitch_Tar);
			if(vision_data.Pitch_Error < 10) Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Max);//发射
			else	Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Max);
			break;
		default:
			//开始模式和自检模式
			break;
	}
//	CAN_Yun.CAN_TransmitForMotor();
}

