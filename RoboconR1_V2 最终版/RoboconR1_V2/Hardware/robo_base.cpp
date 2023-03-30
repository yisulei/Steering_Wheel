/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: robo_base.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "math.h"
#include "robo_base.h"

/* Private variables -------------------------------------------------*/
Base_Typedef 			Robo_Base={{0x201,0x202,0x203},{0x204,0x205,0x206}};
IFR_CAN_ClassDef	CAN_Base;
IFR_PID BaseFollow_PID;

#define num_wheel 3
#define A_car 0.317f
#define PI 3.14159f
#define Half_pi (PI/2)
#define Two_pi (PI*2)
#define MAX_VEL 1.5f
#define MAX_SPIN 5.0f

double V_car;
double angle_car;
double R_car;
float a = 0.0f;
float b = 0.0f;
float angle[5] = {0.0f};
float speed[5] = {0.0f};

float Motor_Tar[8] = {0.0f};
int cir[4];
float MoveMotor_SpeedMix[4] = {0.0f};
const int num_HalfPI = 2048*65*19/10;

void Base_Motor_Init(void)
{
	int i;
	for(i=0;i<3;i++)
	{
		//PID初始化
		Robo_Base.Motor_Move[i].Motor_PID_Init	 			(5.0f,	 0.06f,		0.05f,	10000,	5000,		5000,		0);
		Robo_Base.Motor_Turn[i].MotorPos_PID_Init			(1.0f, 0.0012f,   32.0f, 	 3000,  6000, 10000,		30);
		Robo_Base.Motor_Turn[i].MotorSpeed_PID_Init		(5.0f, 	 0.05f, 	0.06f, 	5000,  6000, 	300000,	0);
		//电机初始化
		Robo_Base.Motor_Turn[i].Motor_PosInit(0);
	}
//	Robo_Base.Motor_Move[0].Motor_PID_Init	 			(8.0f,	 0.06f,		0.05f,	10000,	5000,		5000,		0);
//	Robo_Base.Motor_Move[2].Motor_PID_Init	 			(10.0f,	 0.06f,		0.05f,	10000,	5000,		5000,		0);
	BaseFollow_PID.PID_Init(-1.0f,	0,	0, 660, 6000,	300000,	1);
}
/**
  * Name：TarSpeed_Receive
  * 函数功能：接收上位机发送的X Y W轴的速度, XY单位 米每秒, W单位 弧度每秒
  * 入口参数：
  * 出口参数：
  * Author：liye
  * Date：2022-7-19
  */
void Base_Control(float Vel_Y,float Vel_X,float Vel_W)
{
	Robo_Base.Vel_X = Vel_X / 660.0f * MAX_VEL;
	Robo_Base.Vel_Y = Vel_Y / 660.0f * MAX_VEL;
	Robo_Base.Vel_W = Vel_W / 660.0f * MAX_SPIN;
	ThreeSteering_MoveIK(Robo_Base.Vel_Y,Robo_Base.Vel_X,Robo_Base.Vel_W);
	MotorTarget_Set();
}
/**
  * Name：ThreeSteering_MoveIK
  * 函数功能：三轮舵轮底盘速度逆解 
  * 入口参数：三个轴向的速度
	* 出口参数：底盘运动电机目标速度，转向电机目标绝对角度
  * Author：Yisulei
  * Date：2021-12-04
  */
float Vel_Convert(float Vel)
{
	float motor_vel = Vel * 5 / 8 * 19 * 60 / 0.095f / PI;
	
	return motor_vel;
}



static void ThreeSteering_MoveIK(float Vel_Y,float Vel_X,float Vel_W)
{
	V_car = sqrt(Vel_X * Vel_X + Vel_Y * Vel_Y);
	angle_car = atan2(Vel_X , Vel_Y);

	for(int num = 0; num < num_wheel; num++)
	{
		cir[num] = (int)(Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle/ 8192);
	}
	if(Vel_W == 0 && (Vel_Y != 0 || Vel_X != 0))
	{
		angle[F] = angle_car;
		angle[LB] = angle_car;
		angle[RB] = angle_car;
		speed[F]  = V_car;
		speed[LB] = V_car;
		speed[RB] = V_car;
	}
	else if(Vel_W != 0 && (Vel_Y == 0 && Vel_X == 0))
	{
			angle[F] = Half_pi;
			angle[LB] = PI*1/6;
			angle[RB] = -PI*1/6;
			speed[F] = Vel_W*A_car;
			speed[LB] = -Vel_W*A_car;
			speed[RB] = Vel_W*A_car;
	}
	else if(Vel_W != 0 && (Vel_Y != 0 || Vel_X != 0))
	{
		R_car = abs(V_car / Vel_W);
		b = R_car * sin(angle_car);
		a = R_car * cos(angle_car);
		if(Vel_W<0)
		{
			angle[F] = atan2(-(A_car - b), a );
			angle[LB] = atan2((b + A_car / 2) , (a + A_car * 1.7f / 2)  );
			angle[RB] = atan2((b + A_car / 2) , (a - A_car * 1.7f / 2)  );
			speed[F] = sqrt((b - A_car) * (b - A_car) + a * a) / R_car *abs(V_car);
			speed[LB] = sqrt((b + A_car / 2) * (b + A_car / 2) + (a - A_car * 1.7f / 2) * (a - A_car * 1.7f / 2)) / R_car * abs(V_car);
			speed[RB] = sqrt((b + A_car / 2) * (b + A_car / 2) + (a + A_car * 1.7f / 2) * (a + A_car * 1.7f / 2)) / R_car * abs(V_car);
		}
		else
		{
			angle[F] = atan2(b + A_car, a );
			angle[LB] = atan2((b - A_car / 2) , (a - A_car * 1.7f / 2)  );
			angle[RB] = atan2((b - A_car / 2) , (a + A_car * 1.7f / 2)  );
			speed[F] = sqrt((b + A_car) * (b + A_car) + a * a) / R_car * abs(V_car);
			speed[LB] = sqrt((b - A_car / 2) * (b - A_car / 2) + (a + A_car * 1.7f / 2) * (a + A_car * 1.7f / 2)) / R_car * abs(V_car);
			speed[RB] = sqrt((b - A_car / 2) * (b - A_car / 2) + (a - A_car * 1.7f / 2) * (a - A_car * 1.7f / 2)) / R_car * abs(V_car);
		}
	}
	else
	{
		speed[F] = 0;
		speed[LB] = 0;
		speed[RB] = 0;
	}
	Motor_Tar[0] = Vel_Convert(speed[F]);
	Motor_Tar[1] = Vel_Convert(speed[LB]);
	Motor_Tar[2] = Vel_Convert(speed[RB]);
	Motor_Tar[3] = (Rad_Turn_MotorAngle(angle[F])   + cir[0] /19 * 10/65  * 8192)* 65/10 * 19;
	Motor_Tar[4] = (Rad_Turn_MotorAngle(angle[LB])  + cir[1] /19 * 10/65  * 8192)* 65/10 * 19;
	Motor_Tar[5] = (Rad_Turn_MotorAngle(angle[RB])  + cir[2] /19 * 10/65  * 8192)* 65/10 * 19;//注意减速比为浮点数时候，要先化整
}
/**
  * Name：MotorTarget_Set
	* 函数功能：根据当前角度与目标角度位置关系，计算实际的电机的目标角度和速度
  * 入口参数：底盘逆解得到的运动电机目标速度，转向电机目标绝对角度
	* 出口参数：实际给电机的目标角度和速度
  * Author：Yisulei
  * Date：2021-12-04
  */
static void MotorTarget_Set()
{
	for (int num = 0;num < num_wheel;num++)
	{
		if (Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle >= 0)
		{
			if (((Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle - Motor_Tar[num+num_wheel]) <= num_HalfPI) && ((Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle - Motor_Tar[num+num_wheel]) >= -num_HalfPI ))
			{
				;
			}
			else if (Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle - Motor_Tar[num+num_wheel] > num_HalfPI*3)
			{
				Motor_Tar[num+num_wheel] += 4*num_HalfPI;
			}
			else if (Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle > Motor_Tar[num+num_wheel])
			{
				Motor_Tar[num+num_wheel] += num_HalfPI*2;
				Motor_Tar[num] = -Motor_Tar[num];
			}
			else if (Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle < Motor_Tar[num+num_wheel])
			{
				Motor_Tar[num+num_wheel] -= num_HalfPI*2;
				Motor_Tar[num] = -Motor_Tar[num];
			} 	
		}
		else if (Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle < 0)
		{
			if (((Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle - Motor_Tar[num+num_wheel]) <= num_HalfPI) && ((Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle - Motor_Tar[num+num_wheel]) >= -num_HalfPI ))
			{
				;
			}
			else if (Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle - Motor_Tar[num+num_wheel] < -3*num_HalfPI)
			{
				Motor_Tar[num+num_wheel] -= 4*num_HalfPI;
			}
			else if (Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle > Motor_Tar[num+num_wheel])
			{
				Motor_Tar[num+num_wheel] += 2*num_HalfPI;
				Motor_Tar[num] = -Motor_Tar[num];
			}
			else if (Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle < Motor_Tar[num+num_wheel])
			{
				Motor_Tar[num+num_wheel] -= 2*num_HalfPI;
				Motor_Tar[num] = -Motor_Tar[num];
			} 	
		}
		MoveMotor_SpeedMix[num] = Robo_Base.Motor_Turn[num].Pos_Info.Speed /19 *10/65 * 19;
	
		Robo_Base.Motor_Turn[num].Motor_Pos_Set((int32_t)Motor_Tar[num+num_wheel]);

//		if(abs(Robo_Base.Motor_Turn[num].Pos_Info.Abs_Angle - Robo_Base.Motor_Turn[num].Tar_Pos) <= 500)	
			Robo_Base.Motor_Move[num].Motor_Speed_Set(Motor_Tar[num] - MoveMotor_SpeedMix[num]);
//		else
//			Robo_Base.Motor_Move[num].Motor_Speed_Set(- MoveMotor_SpeedMix[num]);
//		Robo_Base.Motor_Move[num].Motor_Speed_Set(- MoveMotor_SpeedMix[num]);
	}
	CAN_Base.CAN_TransmitForMotor();
}

void duolun_biaoding(void)
{
	for(int i = 0;i<3;i++)
	{
		Robo_Base.Motor_Turn[i].Motor_Pos_Set(0);
		MoveMotor_SpeedMix[i] = Robo_Base.Motor_Turn[i].Pos_Info.Speed /19 *10/65 * 19;
		Robo_Base.Motor_Move[i].Motor_Speed_Set(- MoveMotor_SpeedMix[i]);
	}
	CAN_Base.CAN_TransmitForMotor();
}

static float Rad_Turn_MotorAngle(float angle)
{
	float Motor_Angle = 0;
	Motor_Angle = angle / PI * 180.f / 360.f * 8192.f;
	
	return Motor_Angle;
}


