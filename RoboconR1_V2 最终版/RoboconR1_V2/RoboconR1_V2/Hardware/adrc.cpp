/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: adrc.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-30
  * Description	: 
				ADRC自抗扰控制说白了就是PID的升级版，保留了PID的优点，改良了PID的缺点，其结构和PID一样，ADRC可以被看作三个作
				用效果的结合，分别是TD（跟踪微分器）、ESO（扩张状态观测器）、NLSEF（非线性控制律）。TD是为了防止目标值突变而
				安排的过渡过程；ADRC的灵魂就在于ESO，其作用是观测的是系统的总扰动，使得系统不容易被外界影响；NLSEF是为了改良
				PID直接线性加权（输出=比例+积分+微分）的缺点而引进的非线性控制律，其更符合非线性系统。
  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "adrc.h"

/* Private variables -------------------------------------------------*/

void ADRC_Classdef::ADRC_Init(ADRC_Typedef *ADRC_SET)
{
	memcpy(&ADRC , ADRC_SET , sizeof(ADRC));
}

float ADRC_Classdef::ADRC_Calc(float Target_Value,float Actual_Value)
{
/******************************TD****************************************/
    Process_Para.x1  += ADRC.h*Process_Para.x2;
    Process_Para.x2  += ADRC.h*fhan(Process_Para.x1-Target_Value,Process_Para.x2,ADRC.r,ADRC.h);
/******************************ESO***************************************/
    Process_Para.e = Process_Para.z1 - Actual_Value;
    Process_Para.z1  += ADRC.h*(Process_Para.z2 - ADRC.belta1*Process_Para.e );
    Process_Para.z2  += ADRC.h*(Process_Para.z3 - ADRC.belta2*fal(Process_Para.e,0.5,ADRC.delta) + ADRC.b*Process_Para.output);
    Process_Para.z3  += ADRC.h*(   							- ADRC.belta3*fal(Process_Para.e,0.25,ADRC.delta));
/******************************NLSEF*************************************/
    Process_Para.e1 = Process_Para.x1 - Process_Para.z1;
    Process_Para.e2 = Process_Para.x2 - Process_Para.z2; 
    Process_Para.u0 = ADRC.kp* Process_Para.e1 + ADRC.kd*Process_Para.e2;
/******************************Output************************************/  
    Process_Para.output = Process_Para.u0 - Process_Para.z3/ADRC.b;
  
    return Process_Para.output;
}

static float sign(float x)
{
	if(x>0)
		return 1;
	else if(x<0)
		return -1;
	else
		return 0;
}

static float fhan(float x1,float x2,float r,float h)
{
  float d    = 0,
        a    = 0,
				a0   = 0,
				a1   = 0,
				a2   = 0,
				y    = 0,
				fsg  = 0,
				fhan = 0;

  d    =  r*h*h;
  a0   =  h*x2;
  y    =  x1+a0;
  a1   =  sqrtf(d*(d+8*fabsf(y)));//sqrtf处理float类型的取根号,fabsf处理float类型的取绝对值
  a2   =  a0 + sign(y)*(a1 - d)/2.0f;
	fsg	 =	(sign(y+d)-sign(y-d))/2.0f;
  a    =  (a0+y)*fsg  + a2*(1.0f-fsg);
  fhan =  -r*(a/d)*fsg - r*sign(a)*(1.0f-fsg);
	return  fhan;
}

static float fal(float e,float a,float delta)
{
  float result = 0,fabsf_e = 0;
  fabsf_e = fabsf(e);
  if(delta>=fabsf_e)
    result = e/powf(delta,1.0f-a);
  else if(delta<fabsf_e)
    result = sign(e)*powf(fabsf_e,a);
	return result;    
}

