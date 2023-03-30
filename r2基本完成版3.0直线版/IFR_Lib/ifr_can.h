/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_can.h
  * Version		: v1.3
  * Author		: LiJiawei 
  * Date		: 2021-12-11
  * Description	: 

  *********************************************************************
  */

#ifndef __IFR_CAN_H_
#define __IFR_CAN_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#ifdef HAL_CAN_MODULE_ENABLED
#include "string.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

//获取CAN_ID
#define __IFR_CAN_ID_GET(__HANDLE__) \
((__HANDLE__)->Instance==CAN1?1:\
 (__HANDLE__)->Instance==CAN2?2:0)\

typedef struct
{
	uint8_t Data[8];
	uint8_t pData[8][8];
	uint32_t Transmit_Flag;
	uint32_t Transmit_Finish_Flag;
	CAN_RxHeaderTypeDef Header;
	CAN_TxHeaderTypeDef pHeader[8];
	uint8_t CAN_Analysis_Function_Num;
	void (*CAN_Analysis_Function[5])(uint8_t CAN_ID,uint8_t *Data,uint32_t stdid);
}CAN_LIST_TypeDef;

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void IFR_CAN_Init(CAN_HandleTypeDef *hcan,void (*CAN_Analysis_Function)(uint8_t CAN_ID,uint8_t *Data,uint32_t stdid));
void IFR_CAN_Transmit(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t *pData);
void IFR_CAN_Recevice_Callback(CAN_HandleTypeDef *hcan);
void IFR_CAN_Transmit_Callback(CAN_HandleTypeDef *hcan);
#endif

/* USER CODE END Prototypes */

#endif

