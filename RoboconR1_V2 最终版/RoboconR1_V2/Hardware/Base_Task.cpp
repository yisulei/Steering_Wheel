/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Base_Task.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "Base_Task.h"
//#include "math.h"
/* Private variables -------------------------------------------------*/
float First_Angle=-26;
float Forward_Angle=0;
float Third_Angle=-7;
float Ball_Angle=22;

float Yaw_Tar = 0;
float Base_W;
uint8_t first=0;

int jscope_duolun1;
int jscope_duolun2;
int jscope_duolun3;

int test;
uint8_t Number_of_launches = 0;

void Base_Task(State *state)
{
	jscope_duolun1 = Robo_Base.Motor_Move[0].Pos_Info.Speed;
	jscope_duolun2 = Robo_Base.Motor_Move[1].Pos_Info.Speed;
	jscope_duolun3 = Robo_Base.Motor_Move[2].Pos_Info.Speed;
	switch(*state)
	{
    case ERROR_STATE:
			break;
    case STOP_STATE:
			duolun_biaoding();
			break;
    case MOVE_STATE:
			break;
    case GRAB_STATR:		
			break;
		case MANUAL_STATE:
			Forward_Angle -=(0.001*(DJI_Remote.Chx_Right - 1024));
			Yaw_Tar = Forward_Angle;
			Base_W = BaseFollow_PID.Positional_PID(Yaw_Tar,IMU.quaternion.Absolute_YAW*20);
			Base_Control(-1*(DJI_Remote.Chy_Left - 1024),1*(DJI_Remote.Chx_Left - 1024),Base_W);
			break;
		case MANUALSHOOT_STATE:
			Yaw_Tar = Forward_Angle;
			Base_W = BaseFollow_PID.Positional_PID(Yaw_Tar,IMU.quaternion.Absolute_YAW*20);
			Base_Control(0,0,Base_W);
			break;
    case SHOOT_STATE:
			if(first) 
			{
				Number_of_launches++;
				first = 0;
			}
			if(Number_of_launches == 1)
			{
				Yaw_Tar = First_Angle;
				Base_W = BaseFollow_PID.Positional_PID(Yaw_Tar*20,IMU.quaternion.Absolute_YAW*20);
				Base_Control(0,0,Base_W);
			}
			else if(Number_of_launches == 2) 
			{
				Yaw_Tar = Third_Angle;
				Base_W = BaseFollow_PID.Positional_PID(Yaw_Tar*20,IMU.quaternion.Absolute_YAW*20);
				Base_Control(0,0,Base_W);
			}
			else if(Number_of_launches >= 3) 
			{
				Yaw_Tar = Ball_Angle;
				Base_W = BaseFollow_PID.Positional_PID(Yaw_Tar*20,IMU.quaternion.Absolute_YAW*20);
				Base_Control(0,0,Base_W);
//				Laser_Close();
			}
			break;
		case SMART_STATE:
			Forward_Angle -=(0.005*(DJI_Remote.Chx_Right - 1024));
			Yaw_Tar = Forward_Angle;
			Base_W = BaseFollow_PID.Positional_PID(Yaw_Tar,IMU.quaternion.Absolute_YAW*20);
//			if((-1*(DJI_Remote.Chy_Left - 1024)!=0)||(1*(DJI_Remote.Chx_Left - 1024)!=0)) 
//			{
//				if(distance <= 98) Base_Control(0,1*(DJI_Remote.Chx_Left - 1024),0);
//				else
//					    Base_Control(-1*(DJI_Remote.Chy_Left - 1024)*sin(Yaw_Tar - IMU.quaternion.Absolute_YAW),1*(DJI_Remote.Chx_Left - 1024)*cos(Yaw_Tar - IMU.quaternion.Absolute_YAW),Base_W);
//			}
//			else Base_Control(0,0,Base_W);
//			Base_Control(800,0,300);
//			else 
//			if(distance <=100 && distance >=90)	Base_Control(0,1*(DJI_Remote.Chx_Left - 1024),Base_W);
//	else 
				Base_Control(-1*(DJI_Remote.Chy_Left - 1024),1*(DJI_Remote.Chx_Left - 1024),Base_W);
			first = 1;
			break;
		default:
			//开始模式和自检模式
			break;
	}
	
}


