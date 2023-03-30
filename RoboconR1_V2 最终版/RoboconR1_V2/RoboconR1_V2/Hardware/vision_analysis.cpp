/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: vision_analysis.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "vision_analysis.h"

/* Private variables -------------------------------------------------*/
Vision_Typedef vision_data = {0};
int Sum_Send,Sum_Receive;

void Vision_Data_Process(uint8_t *pData,uint8_t len) 
{     	
	if(pData == 0 || pData[0] != 0x5B)     
	{         
		return;     
	}      
	
	vision_data.IT_look	=	pData[1];
	vision_data.Pitch_Error	=	(int16_t)pData[2]<<8|pData[3];
	vision_data.Yaw_Error	=	(int16_t)pData[4]<<8|pData[5];
	
	vision_data.Delay_ms_vision = pData[6];
	Sum_Send = (int16_t)pData[7]<<8|pData[8];
	
	Sum_Receive = vision_data.IT_look+vision_data.Pitch_Error+vision_data.Yaw_Error + vision_data.Delay_ms_vision;
	if(Sum_Send == Sum_Receive) vision_data.Update_vision = 1;
}


