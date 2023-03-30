/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: imu_analysis.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "imu_analysis.h"

/* Private variables -------------------------------------------------*/
IMU_Typedef IMU;

void IMU_Analysis(uint8_t *pData,uint8_t len)
{ 
	if(pData == NULL || pData[0] != 0x5A|| pData[1] != 0xA5) 
	{
		return;
	}
	if(pData[15]==0xB0)
  {
   IMU.gyro.x=((int16_t)(pData[17]<<8 | pData[16])/10.0f) ;
   IMU.gyro.y =((int16_t)(pData[19]<<8 | pData[18])/10.0f) ;
   IMU.gyro.z  =((int16_t)(pData[21]<<8 | pData[20])/10.0f) ;
  }
  if(pData[29]==0xD0)
  {
   IMU.quaternion.Pitch=((int16_t)(pData[31]<<8 | pData[30])/100.0f) ;
   IMU.quaternion.Roll =((int16_t)(pData[33]<<8 | pData[32])/100.0f) ;
   IMU.quaternion.Yaw  =((int16_t)(pData[35]<<8 | pData[34])/10.0f) ;
	 Absolute_Angle();
  }
}
 
void Absolute_Angle(void)
{
 static float Last_YAW;
 float Angle_Err=IMU.quaternion.Yaw-Last_YAW;
 if(Angle_Err>180.0f) 
  IMU.quaternion.Absolute_YAW+=Angle_Err-360.0f;
 else if(Angle_Err<-180.0f) 
  IMU.quaternion.Absolute_YAW+=Angle_Err+360.0f;
 else 
  IMU.quaternion.Absolute_YAW+=Angle_Err;
 Last_YAW=IMU.quaternion.Yaw;
}


