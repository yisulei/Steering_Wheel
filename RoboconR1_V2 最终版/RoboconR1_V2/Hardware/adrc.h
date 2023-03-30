/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: adrc.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-03-30
  * Description	: 

  *********************************************************************
  */
#ifndef __ADRC_H_
#define __ADRC_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
	 
#ifdef __cplusplus
 }
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef struct
{
    //参数区，这11个就是需要调整的参数
    /****************TD**********/
    float r;        //快速跟踪因子,与跟踪速度正相关，有噪声放大的副作用，可以由h滤波因子来调节
		float h;        //滤波因子,一般为系统调用步长，0.001s==1ms,可以根据滤波效果微调
    /**************ESO**********/
    float b;        //系统系数
    float delta;    //delta为fal（e，alpha，delta）函数的线性区间宽度
    float belta1;   //扩张状态观测器反馈增益1
    float belta2;   //扩张状态观测器反馈增益2
    float belta3;   //扩张状态观测器反馈增益3
    /**************NLSEF*******/
    float alpha1;//
    float alpha2;//
    float kp;//跟踪输入信号增益
    float kd;//跟踪微分信号增益
}ADRC_Typedef;

typedef struct
{
/****************TD*******************/
	float x1 ,//跟踪输入
				x2 ,//跟踪输入的微分
/****************ESO******************/
				e  ,//误差
				z1 ,//跟踪反馈值
				z2 ,//跟踪反馈值的而微分
				z3 ,//跟踪系统的扰动（总扰动）
/**************NLSEF******************/
				e1 ,//z1和x1的误差
				e2 ,//z2和x2的误差
				u0 ,//非线性组合的输出
				output;//输出值
}ADRC_Process_Para;

class ADRC_Classdef
{
	public:
		void ADRC_Init(ADRC_Typedef *ADRC_SET);
		float ADRC_Calc(float Target_Value,float Actual_Value);
	private:
		ADRC_Process_Para Process_Para;
		ADRC_Typedef ADRC;
};

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
static float sign(float x);
static float fhan(float x1,float x2,float r,float h);
static float fal(float e,float alpha,float delta);
/* USER CODE END Prototypes */


#endif
