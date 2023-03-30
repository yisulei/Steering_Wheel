/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_can.h
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-21
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_CAN_H_
#define __IFR_CAN_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#ifdef HAL_CAN_MODULE_ENABLED
#include "ifr_motor.h"
#include "cmsis_os.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#define IFR_CAN1Flag 	1
#define IFR_CAN2Flag 1<<1

class IFR_CAN_ClassDef
{
	public:
		void CAN_Init(CAN_HandleTypeDef *hcan,IFR_DJI_Motor *DJI_Motor);
		void CAN_Init(CAN_HandleTypeDef *hcan,IFR_DJI_Motor *DJI_Motor,osThreadId *TaskHandle);
		void CAN_Transmit(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData);
		void CAN_MultiMessage_Transmit(void);
		void CAN_Recevice(void);
		void CAN_TransmitForMotor(void);
		osThreadId *_TaskHandle;
		void (*CAN_Analysis_Function)(uint8_t *Data,uint32_t stdid);
	private:
		CAN_HandleTypeDef *_hcan;
		IFR_DJI_Motor* DJI_MotorClass[8];
		uint8_t Class_Num;
		uint8_t RxData[8];
		CAN_RxHeaderTypeDef RXHeader;

		uint32_t Transmit_Flag;
		uint32_t Transmit_Finish_Flag;
		uint8_t MotorData[3][8];
		uint8_t TxData[8][8];
		CAN_TxHeaderTypeDef TxHeader[8];
};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void IFR_CAN_Recevice_Callback(CAN_HandleTypeDef *hcan);
void IFR_FreeRTOS_CAN_Recevice_Callback(CAN_HandleTypeDef *_hcan);
void IFR_CAN_Transmit_Callback(CAN_HandleTypeDef *hcan);
static uint8_t IFR_Can_ID_Get(CAN_HandleTypeDef *hcan);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif
#endif

