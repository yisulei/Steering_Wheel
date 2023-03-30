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

#define ARM_UP 	 		-117642
#define ARM_DOWN		 			0

#define WRIST_UP 	 		-73000
#define WRIST_MID 	 	-45000
#define WRIST_DOWN		 -1000

int32_t Arm_Tar = 0;
int32_t Wrist_Tar = 0;
uint8_t	Ball_Flag = 0;

uint8_t Arm_First = 1;

void Arm_Task(State *state)
{
	switch(*state)
	{
    case ERROR_STATE:
			break;
    case STOP_STATE:
			Robo_YunAndArm.Motor_Arm.output = 0;
			Robo_YunAndArm.Motor_Arm.Motor_PosInit(0);
			break;
    case GRAB_STATR:
			break;
    case SHOOT_STATE:
		case MANUALSHOOT_STATE:
			Arm_Tar = ARM_DOWN;
			if(abs(Robo_YunAndArm.Motor_Arm.Pos_Info.Abs_Angle - ARM_DOWN) <= 500)
				Robo_YunAndArm.Motor_Arm.Motor_OutputMax_Set(3000);
			else Robo_YunAndArm.Motor_Arm.Motor_OutputMax_Set(500);
			Robo_YunAndArm.Motor_Arm.Motor_Pos_Set(Arm_Tar);	
			
			if(Arm_First)
			{
				Wrist_Tar -=40;
				if(Wrist_Tar < WRIST_UP)	Wrist_Tar = WRIST_UP;
				if(Wrist_Tar > WRIST_DOWN)	Wrist_Tar = WRIST_DOWN;	
				Robo_YunAndArm.Motor_Wrist.Motor_Pos_Set(Wrist_Tar);
			}
			else 
			{
				Wrist_Tar +=20; 
				if(Wrist_Tar < WRIST_MID)	Wrist_Tar = WRIST_MID;
				if(Wrist_Tar > WRIST_DOWN)	Wrist_Tar = WRIST_DOWN;	
				Robo_YunAndArm.Motor_Wrist.Motor_Pos_Set(Wrist_Tar);
			}
			break;
		case SMART_STATE:
		case MANUAL_STATE:
			if(Robo_YunAndArm.Photoelectric_Sensor.Read() == 1)
			{
				Robo_YunAndArm.Air_Cylinder.High();
				Arm_Tar = ARM_DOWN;
				if(abs(Robo_YunAndArm.Motor_Arm.Pos_Info.Abs_Angle - ARM_DOWN) <= 500)
				{
					Robo_YunAndArm.Motor_Arm.output = 0;
					Robo_YunAndArm.Motor_Arm.Motor_PosInit(0);
				}
				else 
				{
					Robo_YunAndArm.Motor_Arm.Motor_OutputMax_Set(500);
					Robo_YunAndArm.Motor_Arm.Motor_Pos_Set(Arm_Tar);	
				}

				Wrist_Tar +=40;
				if(Wrist_Tar < WRIST_MID)	Wrist_Tar = WRIST_MID;
				if(Wrist_Tar > WRIST_DOWN)	Wrist_Tar = WRIST_DOWN;	
				Robo_YunAndArm.Motor_Wrist.Motor_Pos_Set(Wrist_Tar);
			}
			else if(Robo_YunAndArm.Photoelectric_Sensor.Read() == 0)
			{
				if(Arm_Tar == ARM_UP && Wrist_Tar == WRIST_UP)
				{
					Ball_Flag = 1;
					Robo_YunAndArm.Air_Cylinder.High();
				}
				else
				{
					Robo_YunAndArm.Air_Cylinder.Low();
				}
				Arm_Tar -=80;
				Wrist_Tar -=40;
				if(Arm_Tar < ARM_UP)	Arm_Tar = ARM_UP;
				if(Arm_Tar > ARM_DOWN)	Arm_Tar = ARM_DOWN;			
				Robo_YunAndArm.Motor_Arm.Motor_OutputMax_Set(3000);
				Robo_YunAndArm.Motor_Arm.Motor_Pos_Set(Arm_Tar);			
				if(Wrist_Tar < WRIST_UP)	Wrist_Tar = WRIST_UP;
				if(Wrist_Tar > WRIST_DOWN)	Wrist_Tar = WRIST_DOWN;	
				Robo_YunAndArm.Motor_Wrist.Motor_Pos_Set(Wrist_Tar);
			}
			break;
		default:
			//开始模式和自检模式
			break;
	}
}

