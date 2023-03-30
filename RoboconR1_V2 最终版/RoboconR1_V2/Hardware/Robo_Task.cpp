/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Robo_Task.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "Robo_Task.h"

/* Private variables -------------------------------------------------*/
State state = {START_STATE};

void Change_State(void)
{
  switch(state)
  {
    case START_STATE:
    case CHECK_STATE:
    case ERROR_STATE:
      return;
    case STOP_STATE:
    case MOVE_STATE:
    case GRAB_STATR:
    case SHOOT_STATE:
		case SMART_STATE:
		case MANUAL_STATE:
		case MANUALSHOOT_STATE:
    {
      switch(DJI_Remote.Switch_Left)
      {
        case RC_SWITCH_UP:
					switch(DJI_Remote.Switch_Right)
					{
						case RC_SWITCH_UP:
							state = STOP_STATE;
							break;
						case RC_SWITCH_MIDDLE:
							state = MANUAL_STATE;
							break;
						case RC_SWITCH_DOWN:
							state = MANUALSHOOT_STATE;
							break;
					}
          break;
        case RC_SWITCH_MIDDLE:
					state = SMART_STATE;
          break;
        case RC_SWITCH_DOWN:
					state = SHOOT_STATE;
          break;
      }
    }
  }
}


