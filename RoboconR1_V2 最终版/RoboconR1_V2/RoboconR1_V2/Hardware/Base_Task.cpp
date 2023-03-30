/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Base_Task.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "Base_Task.h"

/* Private variables -------------------------------------------------*/
void Base_Task(State *state)
{
	switch(*state)
	{
    case ERROR_STATE:
			break;
    case STOP_STATE:
			duolun_biaoding();
			break;
    case MOVE_STATE:
			Base_Control(-0.5*(DJI_Remote.Chy_Left - 1024),0.5*(DJI_Remote.Chx_Left - 1024),2*(DJI_Remote.Chx_Right - 1024));
			break;
    case GRAB_STATR:
			break;
    case SHOOT_STATE:
			break;
		default:
			//开始模式和自检模式
			break;
	}
	
}


