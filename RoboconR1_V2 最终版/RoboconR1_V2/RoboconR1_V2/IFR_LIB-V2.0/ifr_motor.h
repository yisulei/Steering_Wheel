/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_motor.h
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-26
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_MOTOR_H_
#define __IFR_MOTOR_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ifr_pid.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

typedef struct
{
  int16_t Speed;
  uint16_t Angle;
  int32_t Abs_Angle;
  uint8_t Temperature;
  int16_t Electric;
  uint16_t Last_Angle;
}Motor_Pos_Info;

class IFR_DJI_Motor
{
	public:
		uint16_t Motor_Num;
		int16_t output;
		Motor_Pos_Info Pos_Info;
		void Motor_PosInit(void);
		void DJI_Motor_Analysis(uint8_t *Data,uint32_t stdid);
	protected:
		IFR_DJI_Motor(uint16_t Num);
};

class IFR_Speed_Motor:public IFR_DJI_Motor
{
	public:
		IFR_Speed_Motor(uint16_t Num);
		void Motor_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone);
		void Motor_Speed_Set(int16_t Speed_Tar);
	private:
		IFR_PID Speed_PID;
		int16_t Tar_Speed;
};

class IFR_Pos_Motor:public IFR_DJI_Motor
{
	public:
		IFR_Pos_Motor(uint16_t Num);
		IFR_Pos_Motor(float (*Offset_MotoFunc)(int32_t Motor_Tar),uint16_t Num);
		void MotorPos_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone);
		void MotorSpeed_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone);
		void Motor_Pos_Set(int32_t Pos_Tar);
		int32_t Tar_Pos;
	private:
		IFR_PID Pos_PID;
		IFR_PID Speed_PID;
		float (*Motor_Offset_MotoFunc)(int32_t Motor_Tar);
};

class IFR_GyroControl_Motor:public IFR_DJI_Motor
{
	public:
		IFR_GyroControl_Motor(uint16_t Num);
		IFR_GyroControl_Motor(float (*Offset_MotoFunc)(int32_t Motor_Tar),uint16_t Num);
		void MotorAngle_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone);
		void MotorSpeed_PID_Init(float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone);
		void IMU_Data_Get(float IMU_Speed,float IMU_Angle);
		void Motor_Angle_Set(int32_t Angle_Tar);
	private:
		IFR_PID Angle_PID;
		IFR_PID Speed_PID;
		float IMU_Speed;
		float IMU_Angle;
		float (*Motor_Offset_MotoFunc)(int32_t Motor_Tar);
		int32_t Tar_Angle;
};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif

