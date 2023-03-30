/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_dji_motor.c
  * Version		: v1.3
  * Author		: LiJiawei 
  * Date		: 2021-12-11
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_dji_motor.h"

/* Private variables -------------------------------------------------*/
DJI_MOTOR_TypeDef DJI_Motor[3][9];
#if USE_HAL_CAN_REGISTER_CALLBACKS
#ifdef HAL_CAN_MODULE_ENABLED
void IFR_DJI_Motor_Init(MOTOR_DATA_TypeDef *Motor_Data,uint8_t CAN_ID,uint8_t Motor_ID)
{
	DJI_Motor[CAN_ID][Motor_ID].Motor_Data=Motor_Data;
}

void IFR_DJI_Motor_Analysis(uint8_t CAN_ID,uint8_t *Data,uint32_t stdid)
{
	if(stdid>=0x201&&stdid<=0x208)
	{
		uint8_t Motor_ID=stdid-0x200;
		if(DJI_Motor[CAN_ID][Motor_ID].Motor_Data==NULL)
			return;
		else
		{
					
			DJI_Motor[CAN_ID][Motor_ID].Motor_Data->Angle=Data[0]<<8|Data[1];
			DJI_Motor[CAN_ID][Motor_ID].Motor_Data->Speed=Data[2]<<8|Data[3];
			DJI_Motor[CAN_ID][Motor_ID].Torque=Data[4]<<8|Data[5];
			DJI_Motor[CAN_ID][Motor_ID].Temperature=Data[6];
			IFR_Absolute_Angle_Analysis(&DJI_Motor[CAN_ID][Motor_ID]);
		}
	}
	else
		return;
}

static void IFR_Absolute_Angle_Analysis(DJI_MOTOR_TypeDef *DJI_Motor)
{
	int16_t angle=0;  
	if(DJI_Motor->Last_Angle_Flag==0)
	{
		DJI_Motor->Last_Angle=DJI_Motor->Motor_Data->Angle;
		DJI_Motor->Last_Angle_Flag=1;
	}
	angle=DJI_Motor->Motor_Data->Angle-DJI_Motor->Last_Angle;
	if(angle >= 4096)
	{
		DJI_Motor->Motor_Data->Absolute_Angle=DJI_Motor->Motor_Data->Absolute_Angle + DJI_Motor->Motor_Data->Angle - DJI_Motor->Last_Angle - 8192;		
	}
	else if(angle <= -4096)
	{
		DJI_Motor->Motor_Data->Absolute_Angle =DJI_Motor->Motor_Data->Absolute_Angle + DJI_Motor->Motor_Data->Angle +8192 - DJI_Motor->Last_Angle;		
	}	
	else
	{
		DJI_Motor->Motor_Data->Absolute_Angle=DJI_Motor->Motor_Data->Absolute_Angle+DJI_Motor->Motor_Data->Angle-DJI_Motor->Last_Angle;
	}		
	DJI_Motor->Last_Angle=DJI_Motor->Motor_Data->Angle;	
}

void IFR_DJI_Motor_Transmit(CAN_HandleTypeDef *_hcan)
{
 CAN_TxHeaderTypeDef TxMessage;
 uint8_t Data[8];
 int8_t CAN_ID=__IFR_CAN_ID_GET(_hcan); 
 
 TxMessage.IDE = CAN_ID_STD;      
 TxMessage.RTR = CAN_RTR_DATA;           
 TxMessage.StdId = 0x200; 
 TxMessage.DLC = 8; 

 for(int i=0;i<4;i++)
 {
  if(DJI_Motor[CAN_ID][i+1].Motor_Data==NULL)
  {
   Data[2*i]=0;
   Data[2*i+1]=0;
  }
  else
  {
   Data[2*i]=(DJI_Motor[CAN_ID][i+1].Motor_Data->Motor_Output>>8)&0xff;
   Data[2*i+1]= DJI_Motor[CAN_ID][i+1].Motor_Data->Motor_Output&0xff;
  }
 }
 
 IFR_CAN_Transmit(_hcan,&TxMessage,Data);

 TxMessage.StdId = 0x1ff;
 
 for(int i=0;i<4;i++)
 {
  if(DJI_Motor[CAN_ID][i+5].Motor_Data==NULL)
  {
   Data[2*i]=0;
   Data[2*i+1]=0;
  }
  else
  {
   Data[2*i]=(DJI_Motor[CAN_ID][i+5].Motor_Data->Motor_Output>>8)&0xff;
   Data[2*i+1]= DJI_Motor[CAN_ID][i+5].Motor_Data->Motor_Output&0xff;
  }
 }
 
 IFR_CAN_Transmit(_hcan,&TxMessage,Data);
}
#endif
#endif
