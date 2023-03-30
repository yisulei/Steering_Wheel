/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Chassis_task.c
  * Version		: v2.0
  * Author		: Yi Sulei
  * Date		: 2021-12-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "Chassis_task.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
#include "imu_analyse.h"

#define distance_fix1 260
#define distance_fix2 220
#define distance_fix3 180
#define distance_fix4 141
#define distance_fix5 110
#define MAX_VEL 1.5
#define MAX_SPIN 5

extern IMU_Typedef IMU;
extern uint8_t	Auto_Catch_Finish;
extern uint8_t target_num;
extern uint8_t Auto_Put_1;
extern uint8_t Auto_Put_2;
extern uint8_t Auto_Catch;
double yaw_tar=0;
int output_w;
PID_TypeDef imu_pid;
int distance_fix = 0;
int speed_now1_1 = 0;
int speed_tar1_1 = 0;

Chassis_Control Chassis_control;

UART_RX_BUFFER Uart3_Rx;
UART_RX_BUFFER Uart2_Rx;
UART_RX_BUFFER Uart6_Rx;
uint32_t  distance1;
uint32_t  distance2;
uint8_t qwe[8]={0};
void analysis1(uint8_t* pdata,uint8_t len)
{
	
qwe[0]=pdata[0];
qwe[1]=pdata[1];
qwe[2]=pdata[2];
qwe[3]=pdata[3];
qwe[4]=pdata[4];
qwe[5]=pdata[5];
qwe[6]=pdata[6];
qwe[7]=pdata[7];	
	if(pdata[2]==0x03)
	  distance1=(pdata[3]<<24)|(pdata[4]<<16)|(pdata[5]<<8)|pdata[6];
}

void analysis2(uint8_t* pdata,uint8_t len)
{
	if(pdata[2]==0x03)
	  distance2=(pdata[3]<<24)|(pdata[4]<<16)|(pdata[5]<<8)|pdata[6];
}
uint8_t BCC(uint8_t* dat,uint16_t len)
{
 uint8_t i;
 uint8_t bcc = 0;
 for(i=0;i<len;i++)
 {
 bcc ^= dat[i];
 }
 return bcc;
}



/* Private variables -------------------------------------------------*/

/**
  * 函数名称：ALL_Init
  * 函数功能：初始化所有外设
  * 入口参数：无
  * 出口参数：无
  * 作者：Yi Sulei
  * 修改日期：2022-01-08
  */
void ALL_Init()
{
	Motor_ALL_Init();//电机
	IFR_UART_Init(&huart1, IFR_DJI_Remote_Analysis);//旧遥控器
	IFR_UART_Init(&huart2, IMU_Analysis);//
	IFR_UART_Init(&huart3, analysis1);//
	IFR_UART_Init(&huart6, analysis2);//
	HAL_Delay(20);	
	IFR_PID_Param_Init(&imu_pid,-1,0,0,660,360,2000,2);
	IFR_CAN_Init(&hcan1,IFR_DJI_Motor_Analysis);//Can1
	IFR_CAN_Init(&hcan2,IFR_DJI_Motor_Analysis);//Can2
	IFR_TIM_Init (&htim2,State_Machine);//状态机的定时器中断
	//IFR_UART_Init(&huart3,IMU_Analysis);//新遥控器
	if (State == NOMAL_STATE)
	{
//		IFR_UART_Init(&huart2, TarSpeed_Receive);//上位机控制接口
	}
	else if (State == TEST_STATE)
	{

	}
	
}

/**
  * 函数名称：Motor_ALL_Init
  * 函数功能：初始化所有电机
  * 入口参数：无
  * 出口参数：无
  * 作者：Yi Sulei  Yang Yige
  * 修改日期：2022-01-08
  */
void Motor_ALL_Init(void)
{
    //设定所有电机的Can口及编号
	//ypy：由于布线需要电机号有所更改，整辆车左侧6电机为can2，右侧为can1
	//终端电阻为每条can总线的1号位和主控
	IFR_DJI_Motor_Init(&Chassis_control.Motor_move[LF].DJI_Motor,2,1);
	IFR_DJI_Motor_Init(&Chassis_control.Motor_move[LB].DJI_Motor,2,2);
	IFR_DJI_Motor_Init(&Chassis_control.Motor_move[RF].DJI_Motor,1,1);
	IFR_DJI_Motor_Init(&Chassis_control.Motor_move[RB].DJI_Motor,1,2);
	
	IFR_DJI_Motor_Init(&Chassis_control.Motor_turn[LF].DJI_Motor,2,3);
	IFR_DJI_Motor_Init(&Chassis_control.Motor_turn[LB].DJI_Motor,2,4);
	IFR_DJI_Motor_Init(&Chassis_control.Motor_turn[RF].DJI_Motor,1,3);
	IFR_DJI_Motor_Init(&Chassis_control.Motor_turn[RB].DJI_Motor,1,4);

	IFR_DJI_Motor_Init(&Chassis_control.Motor_claw[L] .DJI_Motor,2,5);
	IFR_DJI_Motor_Init(&Chassis_control.Motor_claw[R] .DJI_Motor,1,5);
	IFR_DJI_Motor_Init(&Chassis_control.Motor_lift    .DJI_Motor,2,6);
	IFR_DJI_Motor_Init(&Chassis_control.Motor_rotate  .DJI_Motor,1,6);//因为6020电机的标识符是0x204，比其他电机高4个编号，所以这个电机编号设成2的时候，处理的时候会被当成6号电机。

    //设定所有电机的pid参数
	IFR_PID_Param_Init(&Chassis_control.Motor_move[LF].speed_pid,5.0f,0.06f,0.05f,10000,5000,5000,0);
	IFR_PID_Param_Init(&Chassis_control.Motor_move[LB].speed_pid,5.0f,0.06f,0.05f,10000,5000,5000,0);
	IFR_PID_Param_Init(&Chassis_control.Motor_move[RF].speed_pid,5.0f,0.06f,0.05f,10000,5000,5000,0);
	IFR_PID_Param_Init(&Chassis_control.Motor_move[RB].speed_pid,5.0f,0.06f,0.05f,10000,5000,5000,0);
	
	IFR_PID_Param_Init(&Chassis_control.Motor_turn[LF].speed_pid,5.0f, 0.05f,  0.06f, 10000,  6000, 300000,0);
	IFR_PID_Param_Init(&Chassis_control.Motor_turn[LB].speed_pid,5.0f, 0.05f,  0.06f, 10000,  6000, 300000,0);
	IFR_PID_Param_Init(&Chassis_control.Motor_turn[RF].speed_pid,5.0f, 0.05f,  0.06f, 10000,  6000, 300000,0);
	IFR_PID_Param_Init(&Chassis_control.Motor_turn[RB].speed_pid,5.0f, 0.05f,  0.06f, 10000,  6000, 300000,0);

    IFR_PID_Param_Init(&Chassis_control.Motor_turn[LF].pos_pid,0.5, 0, 0, 5000,  6000, 5000,0);
    IFR_PID_Param_Init(&Chassis_control.Motor_turn[LB].pos_pid,0.5, 0, 0, 5000,  6000, 5000,0);
    IFR_PID_Param_Init(&Chassis_control.Motor_turn[RF].pos_pid,0.5, 0, 0, 5000,  6000, 5000,0);
    IFR_PID_Param_Init(&Chassis_control.Motor_turn[RB].pos_pid,0.5, 0, 0, 5000,  6000, 5000,0);

	IFR_PID_Param_Init(&Chassis_control.Motor_claw[L] .speed_pid, 5, 0.05f, 0.05f, 8000, 8000, 8000, 0);
	IFR_PID_Param_Init(&Chassis_control.Motor_claw[R] .speed_pid, 5, 0.05f, 0.05f, 8000, 8000, 8000, 0);
	IFR_PID_Param_Init(&Chassis_control.Motor_lift    .speed_pid, 5, 0.05f, 0.06f, 7000, 8000, 8000, 0);
//	IFR_PID_Param_Init(&Chassis_control.Motor_rotate  .speed_pid, 5, 0.05f, 0.06f, 22000, 15000, 15000, 0);
	IFR_PID_Param_Init(&Chassis_control.Motor_rotate  .speed_pid, 20, 0.035f, 1.0f, 4000, 15000, 15000, 0);

    IFR_PID_Param_Init(&Chassis_control.Motor_claw[L] .pos_pid,0.8f, 0.00f,  0, 8000,  8000, 8000, 0);
    IFR_PID_Param_Init(&Chassis_control.Motor_claw[R] .pos_pid,0.8f, 0.00f,  0, 8000,  8000, 8000, 0);
    IFR_PID_Param_Init(&Chassis_control.Motor_lift    .pos_pid,0.8f, 0.0012f,  32, 8000,  8000, 8000, 0);
    IFR_PID_Param_Init(&Chassis_control.Motor_rotate  .pos_pid,0.8f, 0.00f,  32, 4000,  4000, 4000, 0);
//    IFR_PID_Param_Init(&Chassis_control.Motor_rotate  .pos_pid,-0.8f, -0.015,  -2, 1000,  1000, 450, 0);

}

/**
  * 函数名称：ChassisMove_Control
  * 函数功能：底盘的控制流程，每次进状态机中断都要调用一次，不然底盘不动
  * 入口参数：无
  * 出口参数：无.
  * 作者：Yi Sulei  Yan Peiyang  Yang Yige
  * 修改日期：2022-01-08
  */
void ChassisMove_Control(void)
{
    if     (target_num==1) distance_fix=distance_fix1;
    else if(target_num==2) distance_fix=distance_fix2;
    else if(target_num==3) distance_fix=distance_fix3;
    else if(target_num==4) distance_fix=distance_fix4;
    else if(target_num==5) distance_fix=distance_fix5;
	if(DJI_Remote.Chx_Left >= 1024-660 && DJI_Remote.Chx_Left <= 1024 + 660 ) Chassis_control.Vel_X = -1*(DJI_Remote.Chx_Left - 1024);//这里把遥控器速度与电机转速对应起来（最值）
	if(DJI_Remote.Chy_Left >= 1024-660 && DJI_Remote.Chy_Left <= 1024 + 660 ) Chassis_control.Vel_Y = 1*(DJI_Remote.Chy_Left - 1024);
	if(DJI_Remote.Chx_Right >= 1024-660 && DJI_Remote.Chx_Right <= 1024 + 660 ) Chassis_control.Vel_W = 1*(DJI_Remote.Chx_Right - 1024);
	if(distance1!=0&&distance2!=0&&	Auto_Catch_Finish==1)
	{	
		if (distance1-distance2 <15||distance2 -distance1 <15)
		{			
			Chassis_control.Vel_W =0;
		}
		if ((distance1<=distance_fix)&&(distance2<=distance_fix))
    {
        Chassis_control.Vel_Y =0;
        Auto_Put_1=1;
        Auto_Put_2=0;
        Auto_Catch=0;
    }
	}
//    FourSteering_MoveIK(&Chassis_control);//四轮舵轮底盘速度逆解
//	Motor_Mix();//给所有底盘舵轮的电机赋值速度
//	MotorTarget_Set(&Chassis_control);	//根据当前角度、目标角度位置关系与位置环档位，计算实际的电机的目标角度和速度
//	TEST_PID();
}
//ypy：
//锁住地盘,后退别伤吸盘
void ChassisMove_back(void)
{
	if(DJI_Remote.Chx_Left >= 1024-660 && DJI_Remote.Chx_Left <= 1024 + 660 ) Chassis_control.Vel_X = 0;
	if(DJI_Remote.Chy_Left >= 1024-660 && DJI_Remote.Chy_Left <= 1024 + 660 ) Chassis_control.Vel_Y = -200;
	if(DJI_Remote.Chx_Right >= 1024-660 && DJI_Remote.Chx_Right <= 1024 + 660 ) Chassis_control.Vel_W = 0;
//	FourSteering_MoveIK(&Chassis_control);//四轮舵轮底盘速度逆解
//	Motor_Mix();//给所有底盘舵轮的电机赋值速度
//	MotorTarget_Set(&Chassis_control);	//根据当前角度、目标角度位置关系与位置环档位，计算实际的电机的目标角度和速度
//	TEST_PID();
}
//ypy：锁住不要动
void ChassisMove_lock(void)
{
	if(DJI_Remote.Chx_Left >= 1024-660 && DJI_Remote.Chx_Left <= 1024 + 660 ) Chassis_control.Vel_X = 0;
	if(DJI_Remote.Chy_Left >= 1024-660 && DJI_Remote.Chy_Left <= 1024 + 660 ) Chassis_control.Vel_Y = 0;
	if(DJI_Remote.Chx_Right >= 1024-660 && DJI_Remote.Chx_Right <= 1024 + 660 ) Chassis_control.Vel_W = 0;
//	FourSteering_MoveIK(&Chassis_control);//四轮舵轮底盘速度逆解
//	Motor_Mix();//给所有底盘舵轮的电机赋值速度
//	MotorTarget_Set(&Chassis_control);	//根据当前角度、目标角度位置关系与位置环档位，计算实际的电机的目标角度和速度
//	TEST_PID();
}


//void TEST_PID(void)
//{
//	speed_now1_1 = Chassis_control.Motor_move[LF].DJI_Motor.Speed;
//	speed_tar1_1 = Chassis_control.Motor_move[LF].DJI_Motor.Speed_Tar;
//}

void Chassis_Move(void)
{
	yaw_tar=((((double)(DJI_Remote.Chx_Right - 1024))/660.0f)*MAX_SPIN*57.325f*0.001)*0.05+yaw_tar;
	output_w=IFR_Incremental_PID(&imu_pid,20*yaw_tar,20*IMU.quaternion.Absolute_YAW);
	Chassis_control.Vel_W = output_w;
 Chassis_control.Vel_X = Chassis_control.Vel_X / 660 * MAX_VEL;
 Chassis_control.Vel_Y = Chassis_control.Vel_Y / 660 * MAX_VEL;
 Chassis_control.Vel_W = Chassis_control.Vel_W / 660 * MAX_SPIN;
 FourSteering_MoveIK(&Chassis_control);//四轮舵轮底盘速度逆解
 Motor_Mix();//给所有底盘舵轮的电机赋值速度
 MotorTarget_Set(&Chassis_control); //根据当前角度、目标角度位置关系与位置环档位，计算实际的电机的目标角度和速度
}







 