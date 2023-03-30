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
int16_t Pitch_Max = -130;
int16_t Pitch_Min = 60;
int16_t Pitch_Mid =	50;
//(Pitch_Max-Pitch_Min)/4.0f

#define Clip_Max 	 	 -3600000
#define Clip_Min 				-1000
#define Clip_Mid 	 	 -2900000

int32_t Pitch_Tar = Pitch_Mid;
extern Vision_Typedef vision_data;
extern uint8_t	Ball_Flag;
extern uint8_t Number_of_launches;
extern uint8_t Arm_First;

uint8_t Frictiongear_Flag = 0;
uint16_t Ball_Time = 0;
uint8_t Shoot_Flag = 0;
uint8_t First = 1;
void Yun_Task(State *state)
{
	Robo_YunAndArm.Motor_Pitch.IMU_Data_Get(IMU.gyro.y,IMU.quaternion.Pitch*20);
	switch(*state)
	{
    case ERROR_STATE:
			break;
    case STOP_STATE:
			//归0标定
			Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(0);
			break;
    case MOVE_STATE:
			break;
		case MANUALSHOOT_STATE:
			//RM激光模块
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			//摩擦轮满速
			Open_Frictiongear(2200);
			//模糊锁死
		  if(abs(Robo_YunAndArm.Motor_Pitch.IMU_Angle-Robo_YunAndArm.Motor_Pitch.Tar_Angle)>100)
				Robo_YunAndArm.Motor_Pitch.Motor_OutputMax_Set(4000);
			else Robo_YunAndArm.Motor_Pitch.Motor_OutputMax_Set(2000);

			Robo_YunAndArm.Motor_Pitch.Motor_Angle_Set(Pitch_Tar);
			Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Max);
			break;
    case SHOOT_STATE:
			First = 0;
			//发射完成
			Frictiongear_Flag = 0;
			Shoot_Flag = 0;
			//RM激光模块
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			//摩擦轮满速
			Open_Frictiongear(2200);
			//模糊锁死
		  if(abs(Robo_YunAndArm.Motor_Pitch.IMU_Angle-Robo_YunAndArm.Motor_Pitch.Tar_Angle)>100)
				Robo_YunAndArm.Motor_Pitch.Motor_OutputMax_Set(4000);
			else Robo_YunAndArm.Motor_Pitch.Motor_OutputMax_Set(2000);
			//发射角赋值
			if(Number_of_launches == 1) Robo_YunAndArm.Motor_Pitch.Motor_Angle_Set(Pitch_Min);
			if(Number_of_launches == 2) Robo_YunAndArm.Motor_Pitch.Motor_Angle_Set(Pitch_Mid);
			if(Number_of_launches >= 3) Robo_YunAndArm.Motor_Pitch.Motor_Angle_Set(Pitch_Max);
			//拨球发射
			if(Number_of_launches == 1)
			{
				if(DJI_Remote.Switch_Right == 3)	
				{
					Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Max);
					Arm_First = 0;
				}
				else Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Mid);
			}
			else Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Max);
			break;
		case SMART_STATE:
			//RM激光模块
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			//摩擦轮关闭
			if(Frictiongear_Flag == 1) Open_Frictiongear(2200);
			else if(Frictiongear_Flag == 0) Close_Frictiongear();
			//模糊锁死
			if(abs(Robo_YunAndArm.Motor_Pitch.IMU_Angle-Robo_YunAndArm.Motor_Pitch.Tar_Angle)>100)
				Robo_YunAndArm.Motor_Pitch.Motor_OutputMax_Set(4000);
			else Robo_YunAndArm.Motor_Pitch.Motor_OutputMax_Set(2000);
			//发射角赋值
			Robo_YunAndArm.Motor_Pitch.Motor_Angle_Set(Pitch_Min);
			//球落位判断
			if(Ball_Flag) Ball_Time++;
			//球到位置255ms
			if(Ball_Time == 500) 
			{
				Frictiongear_Flag = 1;
				Ball_Flag = 0;
				Ball_Time = 0;
				Shoot_Flag = 1;
			}
			//已上弹
			if(Shoot_Flag == 1||First == 1) 
			{
				Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Mid);
			}
			//等待上弹
			else Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Min);
			break;
		case MANUAL_STATE:
			//RM激光模块
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			//摩擦轮关闭
			if(Frictiongear_Flag == 1) Open_Frictiongear(2200);
			else if(Frictiongear_Flag == 0) Close_Frictiongear();
			//模糊锁死
			if(abs(Robo_YunAndArm.Motor_Pitch.IMU_Angle-Robo_YunAndArm.Motor_Pitch.Tar_Angle)>100)
				Robo_YunAndArm.Motor_Pitch.Motor_OutputMax_Set(4000);
			else Robo_YunAndArm.Motor_Pitch.Motor_OutputMax_Set(2000);
			//发射角赋值
			Pitch_Tar -=0.01*(DJI_Remote.Chy_Right-1024);
			if(Pitch_Tar < Pitch_Max-20)	Pitch_Tar = Pitch_Max-20;
			if(Pitch_Tar > Pitch_Min)	Pitch_Tar = Pitch_Min;	
			Robo_YunAndArm.Motor_Pitch.Motor_Angle_Set(Pitch_Tar);
			//球落位判断
			if(Ball_Flag) Ball_Time++;
			//球到位置255ms
			if(Ball_Time == 500) 
			{
				Frictiongear_Flag = 1;
				Ball_Flag = 0;
				Ball_Time = 0;
				Shoot_Flag = 1;
			}
			Robo_YunAndArm.Motor_Launch.Motor_Pos_Set(Clip_Min);
			break;
		default:
			//开始模式和自检模式
			break;
	}
}

