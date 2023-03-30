/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR������ʵ����.
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
  * �������ƣ�IFR_PID_Param_Init
  * �������ܣ�PID��ʼ������Pid�ṹ�帳ֵ
  * ��ڲ�����Pid�ṹ�塢Kp��Ki��Kd��������������������������֡�����
  * ���ڲ�������
  * ���ߣ�Li Jiawei
  * �޸����ڣ�2021-11-14
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
  * �������ƣ�IFR_Positional_PID
  * �������ܣ�λ��ʽPID����
  * ��ڲ�����Pid�ṹ�塢Ŀ��ֵ����ǰֵ
  * ���ڲ�������
  * ���ߣ�Li Jiawei
  * �޸����ڣ�2021-11-14
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
  * �������ƣ�IFR_Incremental_PID
  * �������ܣ��ٶ�ʽPID����
  * ��ڲ�����Pid�ṹ�塢Ŀ��ֵ����ǰֵ
  * ���ڲ�������
  * ���ߣ�Li Jiawei
  * �޸����ڣ�2021-11-14
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

