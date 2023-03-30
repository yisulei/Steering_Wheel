/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Arm_Task.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "Arm_Task.h"

/* Private variables -------------------------------------------------*/
extern YunAndArm_Typedef Robo_YunAndArm;

#define ARM_UP 	 		-143213
#define ARM_DOWN				  0

int32_t Arm_Tar = (ARM_UP/2.0f);

void Arm_Task(State *state)
{
	switch(*state)
	{
    case ERROR_STATE:
			Robo_YunAndArm.Motor_Arm.output = 0;
			Close_Frictiongear();
			break;
    case STOP_STATE:
			Robo_YunAndArm.Motor_Arm.Motor_Pos_Set(Arm_Tar);	
			break;
    case MOVE_STATE:
			Arm_Tar -=(DJI_Remote.Chy_Right-1024)*20.0f/660.0f;
//			if(Arm_Tar < ARM_UP)	Arm_Tar = ARM_UP;
			if(Arm_Tar > ARM_DOWN)	Arm_Tar = ARM_DOWN;	
			Robo_YunAndArm.Motor_Arm.Motor_Pos_Set(Arm_Tar);
			break;
    case GRAB_STATR:
			Robo_YunAndArm.Motor_Arm.Motor_Pos_Set(Arm_Tar);	
			break;
    case SHOOT_STATE:
			Robo_YunAndArm.Motor_Arm.Motor_Pos_Set(Arm_Tar);	
			break;
		default:
			//开始模式和自检模式
			break;
	}
}

