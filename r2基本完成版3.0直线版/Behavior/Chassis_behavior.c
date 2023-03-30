/**
  **************************** Copyright ******************************
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *
  *
  * FileName 	: Chassis_behavior.c
  * Version		: v1.0
  * Author		: Yi Sulei   Yang Yige
  * Date		: 2021-12-04
  * Description	: 底盘行为控制代码，包含底盘及机械结构的运动逻辑

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "Chassis_behavior.h"
#include "math.h"
#include "can.h"
#include "Chassis_task.h"

/* Private variables -------------------------------------------------*/

//这些宏定义是用来规定爪子和抬升位置环位置的，如果操作的时候发现放积木的位置不对，可以在这里改。
//#define Claw_pos0 0
//#define Claw_pos1 -270000
//#define Claw_pos2 -540000
//#define Claw_pos3 -810000
//#define Claw_pos4 -1080000
//#define Claw_pos5 -1330000
#define Claw_pos0 -500
#define Claw_pos1 -250000//-210000
#define Claw_pos2 -360000
#define Claw_pos3 -470000
#define Claw_pos4 -640000
#define Claw_pos5 -790000
#define Lift_pos0 25000//初始位置
#define Lift_pos1 900000//最大块
#define Lift_pos2 650000 //1000000
#define Lift_pos3 950000 //1200000
#define Lift_pos4 1200000//1550000
#define Lift_pos5 1500000//1735000
//**********************
#define error_Lift_pos1 200000 //最大块500000
#define error_Lift_pos2 150000 //80000
#define error_Lift_pos3 100000//65000
//上面三个值很扯淡*********************************************************************
#define error_Lift_pos4 40000
#define error_Lift_pos5 25000
#define put_Lift_pos1 350000//最大块
#define put_Lift_pos2 350000//680000//70
#define put_Lift_pos3 680000//1000000
#define put_Lift_pos4 1000000//1350000
#define put_Lift_pos5 1350000//1650000
//ypy:下面这句在我这里没用了
//#define Lift_pos6 1730000
//问题：最小的一块取的时候不吸上面的部分能放上去吗？
//ypy：我的骚活r
#define situation_normal 0
#define situation_back 1
#define situation_stand 2
#define Rotate_pos_90 2048
#define Rotate_pos_180 4096
uint8_t situation = 0;
uint8_t target_num = 1; //目前抓块到第几个了
uint8_t	Auto_Catch_Finish = 0;
int Manual_Agle = 0;
int wait_delay_catch = 0;
int wait_delay_put = 0;
int subtraction_target_num = 0;
uint8_t Air_Rump_State = 0;
uint8_t Cylinder_State = 0;
uint8_t Flag_bit_1 = 0;
uint8_t Flag_bit_2 = 0;
//更新
uint8_t	left_switch_state_1 = 0;
//底盘运动学解算相关宏定义
#define num_wheel 4
#define len_car 0.5f
#define wid_car 0.5f
#define PI 3.14159f
#define Half_pi (PI/2)
#define Two_pi (PI*2)
//#define abs(x) ((x)>0 ? (x):(-(x)))
//ypy：下面的变量我给加上了初始化=0
//机械结构位置环档位变量
int Claw_level = 0;
int Claw_level_set = 0;
int Lift_level = 0;
int Lift_level_set = 0;
int Lift_level_reset = 0;
int Rotate_pos = 0;
//底盘运动学解算相关变量
int num;
double V_car;
double angle_car;
double R_car;
float a = 0.0f;
float b = 0.0f;
float angle[5] = {0.0f};
float speed[5] = {0.0f};
float Motor_Tar[8] = {0.0f};
int cir[4];
uint8_t rx_vel[10];
float MoveMotor_SpeedMix[4] = {0.0f};
extern Chassis_Control Chassis_control;
extern float Angle_add;
extern uint8_t	Auto_Put_flag_2;
extern uint8_t	Auto_Put_flag_1;
extern uint8_t	Auto_Catch_flag;
extern uint8_t	Auto_Put_2;
extern uint8_t	Auto_Put_1;
extern uint8_t	Auto_Catch;
const int num_HalfPI = 252928;


/**
  * Name：TarSpeed_Receive
  * 函数功能：接收上位机发送的X Y W轴的速度
							帧头(EVEN) + V_Y(H+L) + V_X + V_W
  * 入口参数：串口数据、数据长度
  * 出口参数：无
  * Author：Yi Sulei
  * Date：2021-12-04
  */

void TarSpeed_Receive(uint8_t *pData, uint8_t len)
{
    if(pData[0] % 2 == 0)
    {
        Chassis_control.Vel_Y = pData[1] << 8 | pData[2];
        Chassis_control.Vel_X = pData[3] << 8 | pData[4];
        Chassis_control.Vel_W = pData[5] << 8 | pData[6];
    }

}


float Motor_Angle = 0;
float Rad_Turn_MotorAngle(float angle)
{

    Motor_Angle = angle / PI * 180.f / 360.f * 8192.f;

    return Motor_Angle;
}

float Vel_Convert(float Vel)
{
    float motor_vel = Vel * 5 / 8 * 19 * 60 / 0.095f / PI;

    return motor_vel;
}

/**
  * Name：FourSteering_MoveIK
  * 函数功能：四轮舵轮底盘速度逆解
  * 入口参数：三个轴向的速度
	* 出口参数：底盘运动电机目标速度，转向电机目标绝对角度
  * Author：Yi Sulei
  * Date：2021-12-04
  */

void FourSteering_MoveIK(Chassis_Control *Chassis_control)
{


    V_car = sqrt(Chassis_control->Vel_X * Chassis_control->Vel_X + Chassis_control->Vel_Y * Chassis_control->Vel_Y);//获取底盘速度
    angle_car = atan2(Chassis_control->Vel_X, Chassis_control->Vel_Y); //获取底盘角度

    for(int num = 0; num < num_wheel; num++)//获取转向轮转动圈数
    {
        cir[num] = (int)(Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle / 8192);
    }
    //逆解底盘运动电机目标速度、转向电机目标绝对角度
    if(Chassis_control->Vel_W == 0 && (Chassis_control->Vel_Y != 0 || Chassis_control->Vel_X != 0))
    {
        angle[LF] = angle_car;
        angle[LB] = angle_car;
        angle[RF] = angle_car;
        angle[RB] = angle_car;
        speed[LF] = V_car;
        speed[LB] = V_car;
        speed[RF] = V_car;
        speed[RB] = V_car;
    }
    else if(Chassis_control->Vel_W != 0 && (Chassis_control->Vel_Y == 0 && Chassis_control->Vel_X == 0))
    {
        V_car = Chassis_control->Vel_W * sqrt(( wid_car / 2) * ( wid_car / 2) + ( len_car / 2) * ( len_car / 2));
        speed[LF] = V_car;
        speed[LB] = V_car;
        speed[RF] = -V_car;
        speed[RB] = -V_car;
        angle[LF] = -PI / 4;
        angle[LB] = PI / 4;
        angle[RF] = PI / 4;
        angle[RB] = -PI / 4;
    }
    else if(Chassis_control->Vel_W != 0 && (Chassis_control->Vel_Y != 0 || Chassis_control->Vel_X != 0))
    {
        R_car = abs(V_car / Chassis_control->Vel_W);
        b = R_car * sin(-angle_car);
        a = R_car * cos(angle_car);
        if(Chassis_control->Vel_W < 0)
        {
            angle[RB] = atan2((b + len_car / 2), (a + wid_car / 2)  );
            angle[RF] = -atan2((b - len_car / 2), (a + wid_car / 2)  );
            angle[LB] = -atan2((b + len_car / 2), (a - wid_car / 2)  );
            angle[LF] = atan2((b - len_car / 2), (a - wid_car / 2)  );
            speed[RF] = sqrt((a + wid_car / 2) * (a + wid_car / 2) + (b + len_car / 2) * (b + len_car / 2)) / R_car * V_car;
            speed[RB] = sqrt((a + wid_car / 2) * (a + wid_car / 2) + (b - len_car / 2) * (b - len_car / 2)) / R_car * V_car;
            speed[LF] = sqrt((a - wid_car / 2) * (a - wid_car / 2) + (b + len_car / 2) * (b + len_car / 2)) / R_car * V_car;
            speed[LB] = sqrt((a - wid_car / 2) * (a - wid_car / 2) + (b - len_car / 2) * (b - len_car / 2)) / R_car * V_car;
        }
        else
        {
            angle[LF] = -atan2((b + len_car / 2), (a + wid_car / 2)  );
            angle[LB] = -atan2((b - len_car / 2), (a + wid_car / 2)  );
            angle[RF] = -atan2((b + len_car / 2), (a - wid_car / 2)  );
            angle[RB] = -atan2((b - len_car / 2), (a - wid_car / 2)  );
            speed[LF] = sqrt((a + wid_car / 2) * (a + wid_car / 2) + (b + len_car / 2) * (b + len_car / 2)) / R_car * V_car;
            speed[LB] = sqrt((a + wid_car / 2) * (a + wid_car / 2) + (b - len_car / 2) * (b - len_car / 2)) / R_car * V_car;
            speed[RF] = sqrt((a - wid_car / 2) * (a - wid_car / 2) + (b + len_car / 2) * (b + len_car / 2)) / R_car * V_car;
            speed[RB] = sqrt((a - wid_car / 2) * (a - wid_car / 2) + (b - len_car / 2) * (b - len_car / 2)) / R_car * V_car;
        }
    }
    else
    {
        speed[LF] = 0;
        speed[LB] = 0;
        speed[RF] = 0;
        speed[RB] = 0;
    }

    //赋予传动轮及转向轮目标值
    Motor_Tar[0] = Vel_Convert(speed[LF]);
    Motor_Tar[1] = Vel_Convert(speed[LB]);
    Motor_Tar[2] = Vel_Convert(speed[RF]);
    Motor_Tar[3] = Vel_Convert(speed[RB]);
    Motor_Tar[4] = (Rad_Turn_MotorAngle(angle[LF])   + cir[0] / 19 * 10 / 65  * 8192) * 65 / 10 * 19;
    Motor_Tar[5] = (Rad_Turn_MotorAngle(angle[LB])   + cir[1] / 19 * 10 / 65  * 8192) * 65 / 10 * 19;
    Motor_Tar[6] = (Rad_Turn_MotorAngle(angle[RF])   + cir[2] / 19 * 10 / 65  * 8192) * 65 / 10 * 19;
    Motor_Tar[7] = (Rad_Turn_MotorAngle(angle[RB])   + cir[3] / 19 * 10 / 65  * 8192) * 65 / 10 * 19;

}



int qwerty = 35000;
/**
  * Name：MotorTarget_Set
	* 函数功能：根据当前角度与目标角度位置关系，计算实际的电机的目标角度和速度
  * 入口参数：底盘逆解得到的运动电机目标速度，转向电机目标绝对角度
	* 出口参数：实际给电机的目标角度和速度
  * Author：Yi Sulei  Yang Yige
  * Date：2021-12-04
  */

void MotorTarget_Set(Chassis_Control *Chassis_control)
{
    for (    num = 0; num < num_wheel; num++) //给底盘电机赋值
    {
        if (Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle >= 0)
        {
            if (((Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle - Motor_Tar[num + num_wheel]) <= num_HalfPI) && ((Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle - Motor_Tar[num + num_wheel]) >= -num_HalfPI ))
            {
                ;
            }
            else if (Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle - Motor_Tar[num + num_wheel] > num_HalfPI * 3)
            {
                Motor_Tar[num + num_wheel] += 4 * num_HalfPI;
            }
            else if (Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle > Motor_Tar[num + num_wheel])
            {
                Motor_Tar[num + num_wheel] += num_HalfPI * 2;
                Motor_Tar[num] = -Motor_Tar[num];
            }
            else if (Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle < Motor_Tar[num + num_wheel])
            {
                Motor_Tar[num + num_wheel] -= num_HalfPI * 2;
                Motor_Tar[num] = -Motor_Tar[num];
            }
        }
        else if (Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle < 0)
        {
            if (((Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle - Motor_Tar[num + num_wheel]) <= num_HalfPI) && ((Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle - Motor_Tar[num + num_wheel]) >= -num_HalfPI ))
            {
                ;
            }
            else if (Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle - Motor_Tar[num + num_wheel] < -3 * num_HalfPI)
            {
                Motor_Tar[num + num_wheel] -= 4 * num_HalfPI;
            }
            else if (Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle > Motor_Tar[num + num_wheel])
            {
                Motor_Tar[num + num_wheel] += 2 * num_HalfPI;
                Motor_Tar[num] = -Motor_Tar[num];
            }
            else if (Chassis_control->Motor_turn[num].DJI_Motor.Absolute_Angle < Motor_Tar[num + num_wheel])
            {
                Motor_Tar[num + num_wheel] -= 2 * num_HalfPI;
                Motor_Tar[num] = -Motor_Tar[num];
            }
        }
        MoveMotor_SpeedMix[num] = Chassis_control->Motor_turn[num].DJI_Motor.Speed / 19 * 10 / 65 * 19;

        Chassis_control->Motor_turn[num].DJI_Motor.Angle_Tar = Motor_Tar[num + num_wheel];
        Chassis_control->Motor_move[num].DJI_Motor.Speed_Tar = Motor_Tar[num]/*+MoveMotor_SpeedMix[num]*/;
        //Chassis_control->Motor_rotate.DJI_Motor.Angle_Tar=Rotate_pos;

    }
    //根据爪子的位置档位给爪子位置环赋值
    if(Claw_level == 0)
    {
        Chassis_control->Motor_claw[L].DJI_Motor.Angle_Tar = Claw_pos0;
        Chassis_control->Motor_claw[R].DJI_Motor.Angle_Tar = Claw_pos0;
    }
    else if(Claw_level == 1)
    {
        Chassis_control->Motor_claw[L].DJI_Motor.Angle_Tar = Claw_pos1;
        Chassis_control->Motor_claw[R].DJI_Motor.Angle_Tar = Claw_pos1;
    }
    else if(Claw_level == 2)
    {
        Chassis_control->Motor_claw[L].DJI_Motor.Angle_Tar = Claw_pos2;
        Chassis_control->Motor_claw[R].DJI_Motor.Angle_Tar = Claw_pos2;
    }
    else if(Claw_level == 3)
    {
        Chassis_control->Motor_claw[L].DJI_Motor.Angle_Tar = Claw_pos3;
        Chassis_control->Motor_claw[R].DJI_Motor.Angle_Tar = Claw_pos3;
    }
    else if(Claw_level == 4)
    {
        Chassis_control->Motor_claw[L].DJI_Motor.Angle_Tar = Claw_pos4;
        Chassis_control->Motor_claw[R].DJI_Motor.Angle_Tar = Claw_pos4;
    }
    else if(Claw_level == 5)
    {
        Chassis_control->Motor_claw[L].DJI_Motor.Angle_Tar = Claw_pos5;
        Chassis_control->Motor_claw[R].DJI_Motor.Angle_Tar = Claw_pos5;
    }

    //根据抬升的位置档位给抬升位置环赋值
    if(Lift_level == 0) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = qwerty;
    else if(Lift_level == 2) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = Lift_pos1;
    else if(Lift_level == 4) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = Lift_pos2;
    else if(Lift_level == 6) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = Lift_pos3;
    else if(Lift_level == 8) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = Lift_pos4;
    else if(Lift_level == 10) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = Lift_pos5;
    //是立着的或者背面朝上需要上升到这个高度进行旋转
    else if(Lift_level == 11) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = error_Lift_pos1;
    else if(Lift_level == 12) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = error_Lift_pos2;
    else if(Lift_level == 13) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = error_Lift_pos3;
    else if(Lift_level == 14) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = error_Lift_pos4;
    else if(Lift_level == 15) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = error_Lift_pos5;
    //块在放的时候，不能自由落体因此要带他下来一下，这个就是下降位置
    else if(Lift_level == 1) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = put_Lift_pos1;
    else if(Lift_level == 3) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = put_Lift_pos2;
    else if(Lift_level == 5) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = put_Lift_pos3;
    else if(Lift_level == 7) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = put_Lift_pos4;
    else if(Lift_level == 9) Chassis_control->Motor_lift.DJI_Motor.Angle_Tar = put_Lift_pos5;
}


/**
  * Name：Motor_Mix
  * 函数功能： 给所有底盘舵轮的电机赋值速度
  * 入口参数：无
  * 出口参数：无
  * Author：Yi Sulei
  * Date：2022-01-08
  */

void Motor_Mix(void)
{
    for(int num = 0; num < num_wheel; num++ )
    {
        MoveMotor_SpeedMix[num] = Chassis_control.Motor_turn[num].DJI_Motor.Speed / 19 * 10 / 65 * 19;
    }
}


/**
  * Name：Motor_Drive
  * 函数功能：目标值进行pid运算后赋给对应电机
  * 入口参数：无
  * 出口参数：can1 -> motor_move ; can2 -> Motor_turn
  * Author：Yi Sulei
  * Date：2021-12-05
  */

int32_t tar = 0;
int32_t num_ = 4000;
int32_t err = 40000;
int32_t sp_tar = 8000;
int8_t exchange_transmit_flag = 0;
void Motor_Drive(void)
{
    for(int num = 0 ; num < num_wheel ; num++)
    {
        //Chassis_control.Motor_turn[num].DJI_Motor.Angle_Tar += Angle_add ;
        if (abs(Chassis_control.Motor_turn[num].DJI_Motor.Absolute_Angle - Chassis_control.Motor_turn[num].DJI_Motor.Angle_Tar) < err)
        {
            Chassis_control.Motor_turn[num].pos_pid.Error_Max = num_;
        }
        else
        {
            Chassis_control.Motor_turn[num].pos_pid.Error_Max = 13000;
        }

        Chassis_control.Motor_move[num].DJI_Motor.Motor_Output = IFR_Positional_PID(&Chassis_control.Motor_move[num].speed_pid, Chassis_control.Motor_move[num].DJI_Motor.Speed_Tar - MoveMotor_SpeedMix[num], Chassis_control.Motor_move[num].DJI_Motor.Speed) ;

        Chassis_control.Motor_turn[num].DJI_Motor.Speed_Tar = IFR_Positional_PID(&Chassis_control.Motor_turn[num].pos_pid, Chassis_control.Motor_turn[num].DJI_Motor.Angle_Tar, Chassis_control.Motor_turn[num].DJI_Motor.Absolute_Angle) ;
        Chassis_control.Motor_turn[num].DJI_Motor.Motor_Output = IFR_Positional_PID(&Chassis_control.Motor_turn[num].speed_pid, Chassis_control.Motor_turn[num].DJI_Motor.Speed_Tar, Chassis_control.Motor_turn[num].DJI_Motor.Speed) ;
    }
    Chassis_control.Motor_claw[L].DJI_Motor.Speed_Tar = IFR_Positional_PID(&Chassis_control.Motor_claw[L].pos_pid, Chassis_control.Motor_claw[L].DJI_Motor.Angle_Tar, Chassis_control.Motor_claw[L].DJI_Motor.Absolute_Angle) ;
    Chassis_control.Motor_claw[L].DJI_Motor.Motor_Output = IFR_Positional_PID(&Chassis_control.Motor_claw[L].speed_pid, Chassis_control.Motor_claw[L].DJI_Motor.Speed_Tar, Chassis_control.Motor_claw[L].DJI_Motor.Speed) ;
    Chassis_control.Motor_claw[R].DJI_Motor.Speed_Tar = IFR_Positional_PID(&Chassis_control.Motor_claw[R].pos_pid, Chassis_control.Motor_claw[R].DJI_Motor.Angle_Tar, Chassis_control.Motor_claw[R].DJI_Motor.Absolute_Angle) ;
    Chassis_control.Motor_claw[R].DJI_Motor.Motor_Output = IFR_Positional_PID(&Chassis_control.Motor_claw[R].speed_pid, Chassis_control.Motor_claw[R].DJI_Motor.Speed_Tar, Chassis_control.Motor_claw[R].DJI_Motor.Speed) ;
    Chassis_control.Motor_lift.DJI_Motor.Speed_Tar = IFR_Positional_PID(&Chassis_control.Motor_lift.pos_pid, Chassis_control.Motor_lift.DJI_Motor.Angle_Tar, Chassis_control.Motor_lift.DJI_Motor.Absolute_Angle) ;
    Chassis_control.Motor_lift.DJI_Motor.Motor_Output = IFR_Positional_PID(&Chassis_control.Motor_lift.speed_pid, Chassis_control.Motor_lift.DJI_Motor.Speed_Tar, Chassis_control.Motor_lift.DJI_Motor.Speed) ;
    if(IFR_Positional_PID(&Chassis_control.Motor_rotate.pos_pid, Chassis_control.Motor_rotate.DJI_Motor.Angle_Tar, Chassis_control.Motor_rotate.DJI_Motor.Absolute_Angle) >= 50 || IFR_Positional_PID(&Chassis_control.Motor_rotate.pos_pid, Chassis_control.Motor_rotate.DJI_Motor.Angle_Tar, Chassis_control.Motor_rotate.DJI_Motor.Absolute_Angle) <= -30)
        Chassis_control.Motor_rotate.DJI_Motor.Speed_Tar = IFR_Positional_PID(&Chassis_control.Motor_rotate.pos_pid, Chassis_control.Motor_rotate.DJI_Motor.Angle_Tar, Chassis_control.Motor_rotate.DJI_Motor.Absolute_Angle) ;
    else Chassis_control.Motor_rotate.DJI_Motor.Speed_Tar = 0;
    Chassis_control.Motor_rotate.DJI_Motor.Motor_Output = IFR_Positional_PID(&Chassis_control.Motor_rotate.speed_pid, Chassis_control.Motor_rotate.DJI_Motor.Speed_Tar, Chassis_control.Motor_rotate.DJI_Motor.Speed) ;
    IFR_DJI_Motor_Transmit(&hcan1);
    IFR_DJI_Motor_Transmit(&hcan2);

}

/**
  * 函数名称：Base_Reset
  * 函数功能：下电前将所有电机的位置归零
  * 入口参数：无
  * 出口参数：无
  * 作者：Yang Yige
  * 修改日期：2022-06-28
  */

void Base_Reset(void)
{
    //ypy：下面两句我的骚活r，我的自动抓取有用。
    Auto_Catch_flag = 0;
    Auto_Put_flag_1 = 0;
    Auto_Put_flag_2 = 0;
    Auto_Catch = 0;
    Auto_Put_1 = 0;
    Auto_Put_2 = 0;
    wait_delay_catch = 0;
    wait_delay_put = 0;
    Claw_level = 0;
    Lift_level = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); //吸盘伸出
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); //吸盘气泵打开
    for(int i = 0; i < num_wheel; i++)
    {
        MoveMotor_SpeedMix[i] = Chassis_control.Motor_turn[i].DJI_Motor.Speed * 10 / 65;
        Chassis_control.Motor_turn[i].DJI_Motor.Angle_Tar = 0;
        Chassis_control.Motor_move[i].DJI_Motor.Speed_Tar = MoveMotor_SpeedMix[i];
    }
    Chassis_control.Motor_claw[L].DJI_Motor.Angle_Tar = Claw_pos0;
    Chassis_control.Motor_claw[R].DJI_Motor.Angle_Tar = Claw_pos0;
    Chassis_control.Motor_lift.DJI_Motor.Angle_Tar = Lift_pos0;
    Chassis_control.Motor_rotate.DJI_Motor.Angle_Tar = 0;
}

/**
  * 函数名称：Carry_Task
  * 函数功能：自动抓块的流程，简化操作难度
  * 入口参数：抓块的步骤编号
  * 出口参数：无
  * 作者：Yang Yige
  * 修改日期：2022-06-28
  */
//ypy:这个变量调试用

void Carry_Task(int Carry_Step)
{
    if(Carry_Step == 0)
    {
        if (Chassis_control.Motor_lift.DJI_Motor.Absolute_Angle < 35000)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); //吸盘收缩
        }
        Lift_level = Lift_level_reset; //抬升下降
    }
    else if(Carry_Step == 1)
    {
        Claw_level = Claw_level_set; //爪子夹取
    }
    else if(Carry_Step == 2)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); //吸盘伸出
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); //吸盘气泵打开
    }
    else if(Carry_Step == 3)
    {
        Claw_level = 0; //爪子打开
        Lift_level = Lift_level_set; //抬升上升
    }
    else if(Carry_Step == 4)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); //吸盘气泵关闭
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
//ypy：下面的函数都是我的杰作

void Auto_Catch_Task(uint8_t Catch_Flag)
{
    if(Catch_Flag && Auto_Catch_Finish)
    {
        if(Flag_bit_2 == 0)
        {
            Chassis_control.Motor_rotate.DJI_Motor.Angle_Tar -= Rotate_pos_180;
            Manual_Agle -= Rotate_pos_180;
            Flag_bit_2 = 1;
        }
    }
    if(Catch_Flag && !Auto_Catch_Finish)
    {
        if((situation == situation_stand || target_num == 5) && wait_delay_catch < 2000 && wait_delay_catch > (1000 + 500))
        {
            Claw_level = 3;
        }
        else if(wait_delay_catch < (1000 + 500))
        {
            if(situation == situation_stand)
            {
                Claw_level = 5;
            }
            else
            {
                Claw_level = target_num;
            }
        }
        else
        {
            Claw_level = 0;
        }
        switch(situation)//抓块有三种情况：正面向上，反面向上，立着
        {
        /*正面向上*/
        case situation_normal:
            if(wait_delay_catch >= 500 && wait_delay_catch <= 1000)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); //吸盘气泵打开
                Air_Rump_State = GPIO_PIN_SET;

            }
            else if(wait_delay_catch >= 1000 && wait_delay_catch <= 2000)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); //吸盘伸出
                Cylinder_State = GPIO_PIN_SET;
            }
            else if(wait_delay_catch >= 3000)
            {
                //等待气缸走完
                Lift_level = 2 * target_num; //抬升到对应挡位高度
            }
            break;
        /*反面向上*/
        case situation_back:
        {
            if(wait_delay_catch >= 500 && wait_delay_catch <= 1000)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); //吸盘气泵打开
                Air_Rump_State = GPIO_PIN_SET;

            }
            else if(wait_delay_catch >= 1000 && wait_delay_catch <= 2000)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); //吸盘伸出
                Cylinder_State = GPIO_PIN_SET;
            }
            else if(wait_delay_catch >= 2000 && wait_delay_catch <= 3000)
            {
                //等待气缸伸缩完毕
                Lift_level = 2 * target_num; //抬升到对应挡位高度
            }
            else if(wait_delay_catch > 4000)
            {
                //等待抬升完毕
                if(Flag_bit_1 == 0)
                {
                    Chassis_control.Motor_rotate.DJI_Motor.Angle_Tar += Rotate_pos_180;
                    Manual_Agle += Rotate_pos_180;
                    Flag_bit_1 = 1;
                }
            }
            break;
        }
        /*立着*/
        case situation_stand:
        {
            Lift_level = target_num + 10; //抬升到对应高度为下一步转动准备
            if(wait_delay_catch >= 500 && wait_delay_catch <= 1000)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); //吸盘气泵打开
                Air_Rump_State = GPIO_PIN_SET;
            }
            else if(wait_delay_catch >= 1000 && wait_delay_catch <= 2000)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); //吸盘伸出
                Cylinder_State = GPIO_PIN_SET;
            }
            else if(wait_delay_catch >= 3000 && wait_delay_catch < 4000)
            {
                Lift_level = 2 * target_num;
            }
            else if(wait_delay_catch >= 4000)
            {
                //等待伸缩完毕
                if(Flag_bit_2 == 0)
                {
                    Chassis_control.Motor_rotate.DJI_Motor.Angle_Tar += Rotate_pos_90;
                    Manual_Agle += Rotate_pos_90;
                    Flag_bit_2 = 1;
                }
                //抬升到对应挡位高度
            }
            break;
        }
        }
        if(wait_delay_catch >= 6000)
        {
            Task_Flag_Reset();
            Auto_Catch_Finish = 1;
        }
        wait_delay_catch++;
        wait_delay_put = 0;
    }
}

void Auto_Put_Task(uint8_t Put_Flag_1,uint8_t Put_Flag_2)    //自动放块
{
    if(Put_Flag_1 && !Put_Flag_2)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); //吸盘气泵关闭
        Lift_level = (2 * target_num) - 1;
        Air_Rump_State = GPIO_PIN_RESET;
        wait_delay_catch = 0;
        wait_delay_put = 0;
        Auto_Catch_flag = 0;
    }
    else if(Put_Flag_2)
    {
        Chassis_control.Motor_rotate.DJI_Motor.Angle_Tar = 0;
        if(wait_delay_put >= 1500 && wait_delay_put <= 2500)
        {
            Lift_level = 0;
        }
        else if(wait_delay_put >= 2500 && wait_delay_put <= 3000)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); //吸盘收回
            Cylinder_State = GPIO_PIN_RESET;
        }
        else if(wait_delay_put >= 3500)
        {
            Task_Flag_Reset();
            target_num++;//该下一个块了
        }
        wait_delay_catch = 0;
        wait_delay_put++;
        Auto_Catch_Finish = 0;
    }
    //这个模式的底盘控制
    if(Put_Flag_2 && wait_delay_put <= 1000)  ChassisMove_back();
    else ChassisMove_Control();
    //块数
    if(target_num > 5)  target_num = 1;
}

//一切手动
//鉴定为比赛基本寄
////////////////////////////////////////////////////////////////
//函数更新
void Manual_Catch_Put(void)
{
    Task_Flag_Reset(); //进入手动模式就认为上一步的自动抓块失误了
    //左手x轴爪子，y轴抬升
    //升降
    //这里后期可以优化进状态机不要写这里
    if(DJI_Remote.Switch_Left != RC_SWITCH_DOWN)
    {
        if(DJI_Remote.Chy_Left >= 1600)
        {
            Lift_level_set = 1;
        }
        else if(DJI_Remote.Chy_Left <= 500)
        {
            Lift_level_set = -1;
        }
        if((Lift_level_set == 1) && abs(DJI_Remote.Chy_Left - 1024) < 100)
        {
            Lift_level++;
            Lift_level_set = 0;
        }
        else if((Lift_level_set == -1) && abs(DJI_Remote.Chy_Left - 1024) < 100)
        {
            Lift_level--;
            Lift_level_set = 0;
        }
    }
    //爪子
    if(DJI_Remote.Chx_Left >= 1600)
    {
        Claw_level_set = 1;
    }
    else if(DJI_Remote.Chx_Left <= 400)
    {
        Claw_level_set = -1;
    }
    if((Claw_level_set == 1) && abs(DJI_Remote.Chx_Left - 1024) < 100)
    {
        Claw_level++;
        Claw_level_set = 0;
    }
    else if((Claw_level_set == -1) && abs(DJI_Remote.Chx_Left - 1024) < 100)
    {
        Claw_level--;
        Claw_level_set = 0;
    }
    //右手x轴转动，y轴伸缩
    if(DJI_Remote.Chy_Right >= 1600)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, !Cylinder_State); //吸盘伸出
    }
    else if(DJI_Remote.Chy_Right <= 400)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, Cylinder_State); //吸盘收回
    }
    if(DJI_Remote.Chx_Right >= 1600)
    {
        Manual_Agle++;
    }
    else if(DJI_Remote.Chx_Right <= 400)
    {
        Manual_Agle--;
    }
    if(Manual_Agle > 4096)
    {
        Manual_Agle = 4096;
    }
    else if(Manual_Agle < (-4096))
    {
        Manual_Agle = -4096;
    }
    Chassis_control.Motor_rotate.DJI_Motor.Angle_Tar = Manual_Agle;


    if(DJI_Remote.Switch_Left == RC_SWITCH_UP && left_switch_state_1 != RC_SWITCH_UP)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, !Air_Rump_State); //气泵状态反转
    }
    else if(DJI_Remote.Switch_Left == RC_SWITCH_MIDDLE && left_switch_state_1 != RC_SWITCH_UP)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, Air_Rump_State); //气泵状态保持//从自动切换过来以后状态保持
    }
    else if(DJI_Remote.Switch_Left == RC_SWITCH_UP && left_switch_state_1 == RC_SWITCH_UP)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, Air_Rump_State);
    }
    else if(DJI_Remote.Switch_Left == RC_SWITCH_MIDDLE && left_switch_state_1 == RC_SWITCH_UP)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, !Air_Rump_State);
    }
    //切档
    if(DJI_Remote.Switch_Left == RC_SWITCH_DOWN)
    {
        if(DJI_Remote.Chy_Left >= 1600)
        {
            subtraction_target_num = 1;
        }
        else if(DJI_Remote.Chy_Left <= 500)
        {
            subtraction_target_num = -1;
        }
        if((subtraction_target_num == 1) && abs(DJI_Remote.Chy_Left - 1024) < 100)
        {
            target_num++;
            subtraction_target_num = 0;
            Auto_Catch_Finish = 0;
        }
        else if((subtraction_target_num == -1) && abs(DJI_Remote.Chy_Left - 1024) < 100)
        {
            target_num--;
            subtraction_target_num = 0;
            Auto_Catch_Finish = 0;
        }
    }
    else
    {
        subtraction_target_num = 0;
    }
    //这里商量一下是到最大了从头开始循环，还是到头了就保持头不动了，目前先按yyg写的
    if (Claw_level < 0) Claw_level = 0;
    if (Claw_level > 5) Claw_level = 5;
    if (Lift_level > 10)Lift_level = Lift_level - 10;
    if (Lift_level < 0) Lift_level = 0;
    //	为什么升降数量不一样上面的原本是6我改成了5
    //	if (Lift_level_reset<0) Lift_level=5;
    //	if (Lift_level_reset>5) Lift_level=0;
    ChassisMove_lock();
    if(target_num < 1)
    {
        target_num = 5;
    }
}

//判断块的状态
void Judge_state(int ctrl_flag)
{
    //正面朝上
    if(DJI_Remote.Switch_Left == RC_SWITCH_MIDDLE && (ctrl_flag >= 3000 || ctrl_flag == 0))
    {
        situation = situation_normal;
        left_switch_state_1 = RC_SWITCH_MIDDLE;
    }
    //反面朝上
    else if(DJI_Remote.Switch_Left == RC_SWITCH_UP && (ctrl_flag >= 3000 || ctrl_flag == 0))
        //立着
    {
        situation = situation_back;
        left_switch_state_1 = RC_SWITCH_UP;
    }
    else if(DJI_Remote.Switch_Left == RC_SWITCH_DOWN && (ctrl_flag >= 3000 || ctrl_flag == 0))
    {
        situation = situation_stand;
        left_switch_state_1 = RC_SWITCH_DOWN;
    }
}

void Task_Flag_Reset()
{
    wait_delay_put = 0;
    wait_delay_catch = 0;
    Auto_Catch_flag = 0;
    Auto_Put_flag_1 = 0;
    Auto_Put_flag_2 = 0;
    Auto_Catch = 0;
    Auto_Put_1 = 0;
    Auto_Put_2 = 0;
}
