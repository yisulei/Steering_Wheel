/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR������ʵ����.
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
	int16_t    Speed;  	            //�ٶ�
	int16_t    Speed_Tar;           //Ŀ���ٶ�
	int16_t    Angle;               //�Ƕ�
	int32_t    Angle_Tar;           //Ŀ��Ƕ�
	int32_t    Absolute_Angle;      //���ԽǶ�
	int16_t    Motor_Output;        //������
}MOTOR_DATA_TypeDef;


typedef struct
{
	int16_t            Last_Angle;          //�ϴ�λ��
	int16_t            Torque;              //ת��
	int16_t            Temperature;         //�¶�
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
