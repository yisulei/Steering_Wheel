/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_pid.c
  * Version		: v1.3
  * Author		: LiJiawei
  * Date		: 2021-11-14
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_pid.h"

/* Private variables ------------------------------------------------*/


/**
  * 函数名称：IFR_PID_Param_Init
  * 函数功能：PID初始化，给Pid结构体赋值
  * 入口参数：Pid结构体、Kp、Ki、Kd、最大输出、最大错误输出、最大积分、死区
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-11-14
  */
void IFR_PID_Param_Init(PID_TypeDef *Pid,float Kp,float Ki,float Kd,float Output_Max,float Error_Max,float Integral_Max,float Dead_Zone)
{	
	Pid->Kp=Kp;
	Pid->Ki=Ki;                   
	Pid->Kd=Kd;
	Pid->Output_Max=Output_Max;
	Pid->Error_Max=Error_Max;
	Pid->Integral_Max=Integral_Max;
	Pid->Dead_Zone=Dead_Zone;
}

/**
  * 函数名称：IFR_Positional_PID
  * 函数功能：位置式PID处理
  * 入口参数：Pid结构体、目标值、当前值
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-11-14
  */
int IFR_Positional_PID(PID_TypeDef *Pid,float Target_Value,float Actual_Value)
{	
	Pid->Error = Target_Value-Actual_Value;
	if(Pid->Error > Pid->Error_Max)
		Pid->Error = Pid->Error_Max;
	if(Pid->Error < -Pid->Error_Max)
		Pid->Error = -Pid->Error_Max;
	if(abs(Pid->Error) <= Pid->Dead_Zone)
		Pid->Error=0;
	
	Pid->Integral=Pid->Integral+Pid->Error;
	if(Pid->Error*Pid->Integral<0)
		Pid->Integral=0;
	if(Pid->Integral > Pid->Integral_Max)
		Pid->Integral=Pid->Integral_Max;
	if(Pid->Integral < -Pid->Integral_Max)
		Pid->Integral=-Pid->Integral_Max;
	
	Pid->Diffrential=Pid->Error-Pid->Last_Error;
	
	Pid->Output=Pid->Kp*Pid->Error+Pid->Ki*Pid->Integral+Pid->Kd*Pid->Diffrential;
	if(Pid->Output>Pid->Output_Max)
		Pid->Output=Pid->Output_Max;
	if(Pid->Output<-Pid->Output_Max)
		Pid->Output=-Pid->Output_Max;
	
	Pid->Last_Error=Pid->Error;

	return Pid->Output;
}

/**
  * 函数名称：IFR_Incremental_PID
  * 函数功能：速度式PID处理
  * 入口参数：Pid结构体、目标值、当前值
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-11-14
  */
int IFR_Incremental_PID(PID_TypeDef *Pid,float Target_Value,float Actual_Value)
{
	Pid->Error = Target_Value-Actual_Value;
	if(Pid->Error > Pid->Error_Max)
		Pid->Error = Pid->Error_Max;
	if(Pid->Error < -Pid->Error_Max)
		Pid->Error = -Pid->Error_Max;
	if(abs(Pid->Error) <= Pid->Dead_Zone)
		Pid->Error=0;    

	Pid->Output+=Pid->Kp*(Pid->Error-Pid->Last_Error)+Pid->Ki*Pid->Error+Pid->Kd*(Pid->Error-2*Pid->Last_Error+Pid->Last_Last_Error);
	if(Pid->Output>Pid->Output_Max)
		Pid->Output=Pid->Output_Max;
	if(Pid->Output<-Pid->Output_Max)
		Pid->Output=-Pid->Output_Max;
	
	Pid->Last_Last_Error=Pid->Last_Error;
	Pid->Last_Error = Pid->Error;                 
	
	return Pid->Output;
}

