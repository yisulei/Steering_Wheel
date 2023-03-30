/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_dji_motor.h
  * Version		: v1.3
  * Author		: LiJiawei 
  * Date		: 2021-12-11
  * Description	: 

  *********************************************************************
  */

#ifndef __IFR_DJI_MOTOR_H_
#define __IFR_DJI_MOTOR_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "ifr_can.h"

/* USER CODE END Includes */
#ifdef HAL_CAN_MODULE_ENABLED
/* USER CODE BEGIN Private defines */
typedef struct
{
	int16_t    Speed;  	            //速度
	int16_t    Speed_Tar;           //目标速度
	int16_t    Angle;               //角度
	int32_t    Angle_Tar;           //目标角度
	int32_t    Absolute_Angle;      //绝对角度
	int16_t    Motor_Output;        //电机输出
}MOTOR_DATA_TypeDef;


typedef struct
{
	int16_t            Last_Angle;          //上次位置
	int16_t            Torque;              //转矩
	int16_t            Temperature;         //温度
	uint8_t            Last_Angle_Flag;
	MOTOR_DATA_TypeDef *Motor_Data;
}DJI_MOTOR_TypeDef;
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void IFR_DJI_Motor_Init(MOTOR_DATA_TypeDef *Motor_Data,uint8_t CAN_ID,uint8_t Motor_ID);
void IFR_DJI_Motor_Analysis(uint8_t CAN_ID,uint8_t *Data,uint32_t stdid);
static void IFR_Absolute_Angle_Analysis(DJI_MOTOR_TypeDef *DJI_Motor);
void IFR_DJI_Motor_Transmit(CAN_HandleTypeDef *_hcan);
/* USER CODE END Prototypes */

#endif
#endif
