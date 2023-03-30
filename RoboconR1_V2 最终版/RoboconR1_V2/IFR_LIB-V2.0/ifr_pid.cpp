/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_pid.cpp
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-28
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_pid.h"

/* Private variables -------------------------------------------------*/

void IFR_PID::PID_Init(float kp,float ki,float kd,float output_max,float error_max,float integral_max,float dead_zone)
{
	Kp=kp;Ki=ki;Kd=kd;
	Output_Max=output_max;
	Error_Max=error_max;
	Integral_Max=integral_max;
	Dead_Zone=dead_zone;
}

float IFR_PID::Positional_PID(float Target_Value,float Actual_Value)
{
	Error = Target_Value-Actual_Value;
	if(Error >  Error_Max) Error =  Error_Max;
	if(Error < -Error_Max) Error = -Error_Max;
	if(abs(Error) <= Dead_Zone) Error=0;

	Integral=Integral+Error;
	if(Error*Integral<0) 				Integral=0;
	if(Integral > Integral_Max) Integral=Integral_Max;
	if(Integral < -Integral_Max)Integral=-Integral_Max;

	Diffrential=Error-Last_Error;

	Output=Kp*Error+Ki*Integral+Kd*Diffrential;
	if(Output> Output_Max) Output=Output_Max;
	if(Output<-Output_Max) Output=-Output_Max;

	Last_Error=Error;

	return Output;
}

float IFR_PID::Incremental_PID(float Target_Value,float Actual_Value)
{
	Error = Target_Value-Actual_Value;

	if(Error >  Error_Max) Error =  Error_Max;
	if(Error < -Error_Max) Error = -Error_Max;
	if(abs(Error) <= Dead_Zone) Error=0;

	Output+=Kp*(Error-Last_Error)+Ki*Error+Kd*(Error-2*Last_Error+Last_Last_Error);

	if(Output>Output_Max)Output=Output_Max;
	if(Output<-Output_Max)Output=-Output_Max;

	Last_Last_Error=Last_Error;
	Last_Error = Error;

	return Output;
}

void IFR_PID::OutputMax_Set(float output_max)
{
	Output_Max=output_max;
}

