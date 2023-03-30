/**
  **************************** Copyright ******************************
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_state.c
  * Version		: v1.3
  * Author		: Li Jiawei
  * Date		: 2021-12-06
  * Description	: 状态机函数

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_state.h"
#include "ifr_dji_remote.h"
#include "can.h"
#include "Chassis_task.h"
#include "Chassis_behavior.h"
/* Private variables -------------------------------------------------*/

STATE_TypeDef State = CHECK_STATE;

extern Chassis_Control Chassis_control;

int Carry_step;//自动抓取步骤编号
int Remote_flag;//左右摇杆切换flag
int asd;
uint8_t	Auto_Put_flag_2;
uint8_t	Auto_Put_flag_1;
uint8_t	Auto_Catch_flag;
uint8_t	Auto_Put_2;
uint8_t	Auto_Put_1;
uint8_t	Auto_Catch;
extern int Claw_level_set;
extern int Lift_level_set;
extern int Lift_level_reset;
extern int Rotate_pos;
extern int wait_delay_catch;

/**
  * 函数名称：State_Machine
  * 函数功能：状态机主函数，里面写状态机要执行的内容
  * 入口参数：无
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-12-06
  */
void State_Machine(void)
{
    State_Change();
    State_Task();
}
//ypy：这里是yyg原版代码，我给注释了改用我的试试看
/**
  * 函数名称：State_Change
  * 函数功能：状态机切换函数，里面写状态机改变的触发条件
  * 入口参数：无
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-12-06
  */
//static void State_Change(void)
//{
//	switch(State)
//	{
//		case CHECK_STATE:
//		{
//			if(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Switch_Right==RC_SWITCH_MIDDLE)
//				State=TEST_STATE;
//			if(DJI_Remote.Switch_Left==RC_SWITCH_DOWN&&DJI_Remote.Switch_Right==RC_SWITCH_DOWN)
//                State=RESET_STATE;
//			if((DJI_Remote.Switch_Left==RC_SWITCH_UP||DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE)&&DJI_Remote.Switch_Right==RC_SWITCH_UP)
//                State=CARRY_STATE;
//			break;
//		}
//		case ERROR_STATE:
//		{
//
//			break;
//		}
//		case NOMAL_STATE://ypy：由于stop没用上，这个自然也进不去
//		{
//			break;
//		}
//		case STOP_STATE://ypy:鉴定为没用上
//		{
//			if(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Switch_Right==RC_SWITCH_MIDDLE)
//				State=NOMAL_STATE;
//			break;
//		}
//		case TEST_STATE:
//		{
//			if(DJI_Remote.Switch_Left==RC_SWITCH_DOWN&&DJI_Remote.Switch_Right==RC_SWITCH_DOWN)
//                State=RESET_STATE;
//			if((DJI_Remote.Switch_Left==RC_SWITCH_UP||DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE)&&DJI_Remote.Switch_Right==RC_SWITCH_UP)
//                State=CARRY_STATE;
//			break;
//		}
//        case RESET_STATE:
//		{
//			if(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Switch_Right==RC_SWITCH_MIDDLE)
//                State=TEST_STATE;
//			if((DJI_Remote.Switch_Left==RC_SWITCH_UP||DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE)&&DJI_Remote.Switch_Right==RC_SWITCH_UP)
//                State=CARRY_STATE;
//			break;
//		}
//        case CARRY_STATE:
//		{
//			if(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Switch_Right==RC_SWITCH_MIDDLE)
//                State=TEST_STATE;
//			if(DJI_Remote.Switch_Left==RC_SWITCH_DOWN&&DJI_Remote.Switch_Right==RC_SWITCH_DOWN)
//                State=RESET_STATE;
//			break;
//		}
//	}
//}


///**
//  * 函数名称：State_Task
//  * 函数功能：状态机内容函数，里面写状态机里面具体要执行的内容
//  * 入口参数：无
//  * 出口参数：无
//  * 作者：Li Jiawei
//  * 修改日期：2021-12-06
//  */
//static void State_Task(void)
//{
//	switch(State)
//	{
//		case CHECK_STATE:
//		{
//
//			break;
//		}
//		case NOMAL_STATE:
//		{

//			break;
//		}
//		case ERROR_STATE:
//		{
//
//			break;
//		}
//		case STOP_STATE:
//		{

//			break;
//		}
//		case TEST_STATE:
//		{
//			if(DJI_Remote.Chx_Left >= 1024-660 && DJI_Remote.Chx_Left <= 1024 + 660 ) Chassis_control.Vel_X = -0.5*(DJI_Remote.Chx_Left - 1024);//这里把遥控器速度与电机转速对应起来（最值）
//			if(DJI_Remote.Chy_Left >= 1024-660 && DJI_Remote.Chy_Left <= 1024 + 660 ) Chassis_control.Vel_Y = 0.5*(DJI_Remote.Chy_Left - 1024);
//			if(DJI_Remote.Chx_Right >= 1024-660 && DJI_Remote.Chx_Right <= 1024 + 660 ) Chassis_control.Vel_W = 0.5*(DJI_Remote.Chx_Right - 1024);
//			ChassisMove_Control();
//			break;
//		}
//		case RESET_STATE:
//		{
//            Base_Reset();
//			break;
//		}
//		case CARRY_STATE:
//		{
//            if(DJI_Remote.Switch_Left==RC_SWITCH_UP&&DJI_Remote.Chy_Right >= 1600)
//            {
//                while(DJI_Remote.Switch_Left==RC_SWITCH_UP&&DJI_Remote.Chy_Right >= 1600){}
//                Claw_level_set++;
//            }
//            if(DJI_Remote.Switch_Left==RC_SWITCH_UP&&DJI_Remote.Chy_Right <= 400)
//            {
//                while(DJI_Remote.Switch_Left==RC_SWITCH_UP&&DJI_Remote.Chy_Right <= 400){}
//                Claw_level_set--;
//            }
//            if(DJI_Remote.Switch_Left==RC_SWITCH_UP&&DJI_Remote.Chy_Left >= 1600)
//            {
//                while(DJI_Remote.Switch_Left==RC_SWITCH_UP&&DJI_Remote.Chy_Left >= 1600){}
//                Lift_level_set++;
//            }
//            if(DJI_Remote.Switch_Left==RC_SWITCH_UP&&DJI_Remote.Chy_Left <=400)
//            {
//                while(DJI_Remote.Switch_Left==RC_SWITCH_UP&&DJI_Remote.Chy_Left <=400){}
//                Lift_level_set--;
//            }
//						//////////////////////////////////////////////////////////////////////////////////////
//            if(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Chy_Right >= 1600)
//            {
//                while(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Chy_Right >= 1600){}
//                Rotate_pos=Rotate_pos-1900;
//            }
//            if(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Chy_Right <= 400)
//            {
//                while(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Chy_Right <= 400){}
//                Rotate_pos=Rotate_pos+1900;
//            }
//            if(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Chy_Left >= 1600)
//            {
//                while(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Chy_Left >= 1600){}
//                Lift_level_reset++;
//            }
//            if(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Chy_Left <=400)
//            {
//                while(DJI_Remote.Switch_Left==RC_SWITCH_MIDDLE&&DJI_Remote.Chy_Left <=400){}
//                Lift_level_reset--;
//            }

//            if(DJI_Remote.Chx_Right >= 1600 && Remote_flag==0)
//            {
//                Carry_step++;
//                Remote_flag=1;
//            }
//            if(DJI_Remote.Chx_Right <= 400 && Remote_flag==1)
//            {
//                Carry_step++;
//                Remote_flag=0;
//            }
//            if(DJI_Remote.Chx_Left >= 1600 && Remote_flag==0)
//            {
//                Carry_step--;
//                Remote_flag=1;
//            }
//            if(DJI_Remote.Chx_Left <= 400 && Remote_flag==1)
//            {
//                Carry_step--;
//                Remote_flag=0;
//            }
//            if (Carry_step<0) Carry_step=4;
//            if (Carry_step>4) Carry_step=0;
//            if (Claw_level_set<0) Claw_level_set=5;
//            if (Claw_level_set>5) Claw_level_set=0;
//            if (Lift_level_set<0) Lift_level_set=6;
//            if (Lift_level_set>6) Lift_level_set=0;
//            if (Lift_level_reset<0) Lift_level_reset=5;
//            if (Lift_level_reset>5) Lift_level_reset=0;
//						ChassisMove_Control();
//            Carry_Task(Carry_step);
//			break;
//		}
//	}
//	//ypy：ifr_dji_motor.c.h和ifr_can.c.h都更新了一版，李嘉伟提供的
//    Motor_Drive();
//}

//ypy：下面是我写的状态机
/**
  * 函数名称：State_Change
  * 函数功能：状态机切换函数，里面写状态机改变的触发条件
  * 入口参数：无
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-12-06
  */
static void State_Change(void)
{
    switch(State)
    {
    case CHECK_STATE:
    {
        if(DJI_Remote.Switch_Right == RC_SWITCH_MIDDLE)
            State = TEST_STATE;

        if(DJI_Remote.Switch_Right == RC_SWITCH_DOWN)
            State = RESET_STATE;

        if(DJI_Remote.Switch_Right == RC_SWITCH_UP)
            State = NOMAL_STATE;
        break;
    }
    case ERROR_STATE:
    {
        break;
    }
    //手动模式
    case NOMAL_STATE:
    {
        if(DJI_Remote.Switch_Right == RC_SWITCH_MIDDLE)
            State = TEST_STATE;

        if(DJI_Remote.Switch_Right == RC_SWITCH_DOWN)
            State = RESET_STATE;
        break;
    }
    case STOP_STATE://ypy:鉴定为没用上
    {
        break;
    }
    //底盘和自动抓块模式
    case TEST_STATE:
    {
        if(DJI_Remote.Switch_Right == RC_SWITCH_DOWN)
            State = RESET_STATE;

        if(DJI_Remote.Switch_Right == RC_SWITCH_UP)
            State = NOMAL_STATE;
        break;
    }
    case RESET_STATE:
    {
        if(DJI_Remote.Switch_Right == RC_SWITCH_MIDDLE)
            State = TEST_STATE;

        if(DJI_Remote.Switch_Right == RC_SWITCH_UP)
            State = NOMAL_STATE;
        break;
    }
    case CARRY_STATE:
    {
        break;
    }
    }
}

/**
  * 函数名称：State_Task
  * 函数功能：状态机内容函数，里面写状态机里面具体要执行的内容
  * 入口参数：无
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-12-06
  */
static void State_Task(void)
{
    switch(State)
    {
    //上电一定先进第一个模式
    case CHECK_STATE:
    {
        break;
    }
    case NOMAL_STATE:
    {
        Manual_Catch_Put();
        break;
    }
    case ERROR_STATE:
    {
        break;
    }
    case STOP_STATE:
    {

        break;
    }
    //地盘加自动抓快模式
    case TEST_STATE:
    {
        asd = wait_delay_catch;
        //自动抓块
        //一下的if elseif if elseif 是一个类似判断推杆，推上去松手下来一次行程
        //判断推杆推上去了//这里if else有顺序不能乱换
        if(DJI_Remote.Chy_Right >= 1600)
        {
            Auto_Catch_flag = 1;
            Auto_Put_flag_1 = 0;
            Auto_Put_flag_2 = 0;
        }
        else if(Auto_Put_1 && DJI_Remote.Chy_Right <= 400)
        {
            Auto_Catch_flag = 0;
            Auto_Put_flag_1 = 0;
            Auto_Put_flag_2 = 1;
        }
        else if(!Auto_Put_1 && DJI_Remote.Chy_Right <= 400)
        {
            Auto_Catch_flag = 0;
            Auto_Put_flag_1 = 1;
            Auto_Put_flag_2 = 0;
        }
        //判断推杆归位了
        if((Auto_Catch_flag) && abs(DJI_Remote.Chy_Right - 1024) < 100)
        {
            Auto_Catch = 1;
            Auto_Put_1 = 0;
            Auto_Put_2 = 0;
        }
        else if((Auto_Put_flag_1) && abs(DJI_Remote.Chy_Right - 1024) < 100)
        {
            Auto_Catch = 0;
            Auto_Put_1 = 1;
        }
        else if((Auto_Put_flag_2) && abs(DJI_Remote.Chy_Right - 1024) < 100)
        {
            Auto_Catch = 0;
            Auto_Put_1 = 1;
            Auto_Put_2 = 1;
        }
        Judge_state(wait_delay_catch);
        Auto_Catch_Task(Auto_Catch);
        Auto_Put_Task(Auto_Put_1,Auto_Put_2);
        break;
    }
    case RESET_STATE:
    {
        Base_Reset();
        break;
    }
    case CARRY_STATE:
    {
        break;
    }
    }
    Chassis_Move();
    //ypy：ifr_dji_motor.c.h和ifr_can.c.h都更新了一版，李嘉伟提供的
    Motor_Drive();
}


