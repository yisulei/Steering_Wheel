/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_motor.cpp
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-26
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_motor.h"

/* Private variables -------------------------------------------------*/

IFR_DJI_Motor::IFR_DJI_Motor(uint16_t Num)
{
	Motor_Num = Num;
	Pos_Info.Last_Angle = 65535;
}

void IFR_DJI_Motor::Motor_PosInit(int32_t PosInit)
{
	Pos_Info.Abs_Angle = PosInit;
	Pos_Info.Last_Angle = 65535;
}

void IFR_DJI_Motor::DJI_Motor_Analysis(uint8_t *Data,uint32_t stdid)
{
	if(stdid == Motor_Num)
	{
		Pos_Info.Angle=(uint16_t)Data[0]<<8|Data[1];
		Pos_Info.Speed=(uint16_t)Data[2]<<8|Data[3];
		Pos_Info.Electric=(uint16_t)Data[4]<<8|Data[5];
		Pos_Info.Temperature=Data[6];
		
		if(Pos_Info.Last_Angle == 65535) Pos_Info.Last_Angle = Pos_Info.Angle;
		if (Pos_Info.Speed >5 || Pos_Info.Speed <-5)
		{
			int16_t Error=Pos_Info.Angle-Pos_Info.Last_Angle;
			Pos_Info.Abs_Angle += Error;
			if (Error < -4096) Pos_Info.Abs_Angle += 8192;
			else if (Error > 4096)  Pos_Info.Abs_Angle -= 8192;
		}Pos_Info.Last_Angle=Pos_Info.Angle;
	}
}

IFR_Speed_Motor::IFR_Speed_Motor(uint16_t Num):IFR_DJI_Motor(Num){ }

void IFR_Speed_Motor::Motor_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone)
{
	Speed_PID.PID_Init(Kp,Ki,Kd,Output_Max,Error_Max,Integral_Max,Dead_Zone);
}

void IFR_Speed_Motor::Motor_Speed_Set(int16_t Speed_Tar)
{
	Tar_Speed = Speed_Tar;
	output = Speed_PID.Positional_PID(Speed_Tar,Pos_Info.Speed);
}

IFR_Pos_Motor::IFR_Pos_Motor(uint16_t Num):IFR_DJI_Motor(Num){}

IFR_Pos_Motor::IFR_Pos_Motor(float (*Offset_MotoFunc)(int32_t Motor_Tar),uint16_t Num):IFR_DJI_Motor(Num)
{
	Motor_Offset_MotoFunc = Offset_MotoFunc;
}

void IFR_Pos_Motor::MotorPos_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone)
{
	Pos_PID.PID_Init(Kp,Ki,Kd,Output_Max,Error_Max,Integral_Max,Dead_Zone);
}

void IFR_Pos_Motor::MotorSpeed_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone)
{
	Speed_PID.PID_Init(Kp,Ki,Kd,Output_Max,Error_Max,Integral_Max,Dead_Zone);
}

void IFR_Pos_Motor::Motor_Pos_Set(int32_t Pos_Tar)
{
	Tar_Pos = Pos_Tar;
	int16_t Speed_Tar = Pos_PID.Positional_PID(Tar_Pos,Pos_Info.Abs_Angle);
	output = Speed_PID.Positional_PID(Speed_Tar,Pos_Info.Speed);
	if(Motor_Offset_MotoFunc !=NULL) output +=(*Motor_Offset_MotoFunc)(Tar_Pos);
}

void IFR_Pos_Motor::Motor_OutputMax_Set(float Output_Max)
{
	Pos_PID.OutputMax_Set(Output_Max);
}

IFR_GyroControl_Motor::IFR_GyroControl_Motor(uint16_t Num):IFR_DJI_Motor(Num){}

IFR_GyroControl_Motor::IFR_GyroControl_Motor(float (*Offset_MotoFunc)(int32_t Motor_Tar),uint16_t Num):IFR_DJI_Motor(Num)
{
	Motor_Offset_MotoFunc = Offset_MotoFunc;
}

void IFR_GyroControl_Motor::MotorAngle_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone)
{
	Angle_PID.PID_Init(Kp,Ki,Kd,Output_Max,Error_Max,Integral_Max,Dead_Zone);
}

void IFR_GyroControl_Motor::MotorSpeed_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone)
{
	Speed_PID.PID_Init(Kp,Ki,Kd,Output_Max,Error_Max,Integral_Max,Dead_Zone);
}

void IFR_GyroControl_Motor::IMU_Data_Get(float imu_speed,float imu_angle)
{
	IMU_Speed = imu_speed;
	IMU_Angle = imu_angle;
}

void IFR_GyroControl_Motor::Motor_Angle_Set(int32_t Angle_Tar)
{
	Tar_Angle = Angle_Tar;
	int16_t Speed_Tar = Angle_PID.Positional_PID(Tar_Angle,IMU_Angle);
	output = Speed_PID.Positional_PID(Speed_Tar,IMU_Speed);
	if(Motor_Offset_MotoFunc !=NULL) output +=(*Motor_Offset_MotoFunc)(Tar_Angle);
}

void IFR_GyroControl_Motor::Motor_OutputMax_Set(float Output_Max)
{
	Speed_PID.OutputMax_Set(Output_Max);
}

