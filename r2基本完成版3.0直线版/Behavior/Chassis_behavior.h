/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR������ʵ����.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Chassis_behavior.h
  * Version		: v2.0
  * Author		: ������ 
  * Date		: 2021-12-04
  * Description	: 

  *********************************************************************
  */

#ifndef __CHASSIS_BEHAVIOR_H_
#define __CHASSIS_BEHAVIOR_H_



/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ifr_lib.h"

#define LF 0
#define LB 1
#define RF 2
#define RB 3
#define F 4
#define L 0
#define R 1

typedef struct Motor_Control
{
	MOTOR_DATA_TypeDef DJI_Motor ; //各电机传入的信息
	PID_TypeDef speed_pid ; //速度环pid
	PID_TypeDef pos_pid ; //位置环pid
}Motor_Control;

typedef struct Chassis_Control//底盘的电机控制
{
	Motor_Control Motor_turn[5];//转向轮
	Motor_Control Motor_move[5];//传动轮
	Motor_Control Motor_claw[2];//爪子
	Motor_Control Motor_lift;   //抬升
	Motor_Control Motor_rotate; //旋转
	double Vel_Y;//底盘Y轴位置
	double Vel_X;//底盘X轴位置
	double Vel_W;//底盘旋转轴位置
}Chassis_Control;


void TarSpeed_Receive(uint8_t *pData,uint8_t len);

void FourSteering_MoveIK(Chassis_Control* Chassis_control);

void MotorTarget_Set(Chassis_Control* Chassis_control);

void Motor_Drive(void);

int PID_pos(PID_TypeDef *Pid,float Target_Value,float Actual_Value);

void Motor_Mix(void);

float Rad_Turn_MotorAngle(float angle);

void Base_Reset(void);

void Carry_Task(int Carry_Step);
//ypy������ĺ��������ҵĽ���
void Manual_Catch_Put(void);

void Auto_Catch_Put(int dji_remote_chy_left);

void Judge_state(int ctrl_flag);

void Auto_Catch_Task(uint8_t Catch_Flag);

void Auto_Put_Task(uint8_t Auto_Put_1,uint8_t Auto_Put_2);

void Task_Flag_Reset();

#endif



