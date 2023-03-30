/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_can.cpp
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-21
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_can.h"
/* Private variables -------------------------------------------------*/
#ifdef HAL_CAN_MODULE_ENABLED
#if USE_HAL_CAN_REGISTER_CALLBACKS
IFR_CAN_ClassDef *CAN_Pointer[3] = {0};
/* Private function prototypes -----------------------------------------------*/
void IFR_CAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan,IFR_DJI_Motor *DJI_Motor)
{
	_hcan = hcan;
	CAN_Pointer[IFR_Can_ID_Get(hcan)] = this;

	DJI_MotorClass[Class_Num] = DJI_Motor;			Class_Num++;

  CAN_FilterTypeDef sFilterConfig;
	if(hcan->Instance == CAN1) sFilterConfig.FilterBank=0,sFilterConfig.SlaveStartFilterBank=14;
	else sFilterConfig.FilterBank=14,sFilterConfig.SlaveStartFilterBank=14;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);

	HAL_CAN_RegisterCallback(hcan,HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,IFR_CAN_Recevice_Callback);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(hcan);
}

void IFR_CAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan,IFR_DJI_Motor *DJI_Motor,osThreadId *TaskHandle)
{
	_hcan = hcan;_TaskHandle = TaskHandle;
	CAN_Pointer[IFR_Can_ID_Get(hcan)] = this;

	DJI_MotorClass[Class_Num] = DJI_Motor;			Class_Num++;

  CAN_FilterTypeDef sFilterConfig;
	if(hcan->Instance == CAN1) sFilterConfig.FilterBank=0,sFilterConfig.SlaveStartFilterBank=14;
	else sFilterConfig.FilterBank=14,sFilterConfig.SlaveStartFilterBank=14;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);

	HAL_CAN_RegisterCallback(hcan,HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,IFR_FreeRTOS_CAN_Recevice_Callback);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(hcan);
}

void IFR_CAN_ClassDef::CAN_Transmit(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
	uint32_t TxMailbox;
	if(HAL_CAN_GetTxMailboxesFreeLevel(_hcan)>0)
	{
		HAL_CAN_AddTxMessage(_hcan,pHeader,pData,&TxMailbox);
	}
	else
	{
		TxHeader[Transmit_Flag%8]=*pHeader;
		for(int i=0;i<pHeader->DLC;i++) TxData[Transmit_Flag%8][i]=pData[i];
		Transmit_Flag++;
		if(Transmit_Flag-Transmit_Finish_Flag>8)
		{
			Error_Handler();
		}
		HAL_CAN_ActivateNotification(_hcan,CAN_IT_TX_MAILBOX_EMPTY);
	}
}

void IFR_CAN_ClassDef::CAN_Recevice(void)
{
	uint8_t i;
	if(HAL_CAN_GetRxMessage(_hcan,CAN_RX_FIFO0,&RXHeader,RxData) == HAL_OK)
	{
		for(i=0;i<Class_Num;i++)
			if(DJI_MotorClass[i]!=NULL)
				(DJI_MotorClass[i]->DJI_Motor_Analysis)(RxData,RXHeader.StdId);
		if(CAN_Analysis_Function != NULL) (*CAN_Analysis_Function)(RxData,RXHeader.StdId);
	}
}

void IFR_CAN_ClassDef::CAN_MultiMessage_Transmit(void)
{
	uint32_t TxMailbox;
	if(Transmit_Flag>Transmit_Finish_Flag)
	{
		HAL_CAN_AddTxMessage(_hcan,&TxHeader[Transmit_Finish_Flag%8],TxData[Transmit_Finish_Flag%8],&TxMailbox);
		Transmit_Finish_Flag++;
	}
	else HAL_CAN_DeactivateNotification(_hcan,CAN_IT_TX_MAILBOX_EMPTY);
}

void IFR_CAN_Recevice_Callback(CAN_HandleTypeDef *_hcan)
{
	IFR_CAN_ClassDef* pCAN_Receive = CAN_Pointer[IFR_Can_ID_Get(_hcan)];
	pCAN_Receive->CAN_Recevice();
}

void IFR_FreeRTOS_CAN_Recevice_Callback(CAN_HandleTypeDef *hcan)
{
	uint8_t CAN_ID = IFR_Can_ID_Get(hcan);
	IFR_CAN_ClassDef* pCAN_Receive = CAN_Pointer[CAN_ID];
	switch(CAN_ID)
	{
		case 1:
			osSignalSet(*(pCAN_Receive->_TaskHandle),IFR_CAN1Flag);
			break;
		case 2:
			osSignalSet(*(pCAN_Receive->_TaskHandle),IFR_CAN2Flag);
			break;
	}
}

void IFR_CAN_Transmit_Callback(CAN_HandleTypeDef *_hcan)
{
	IFR_CAN_ClassDef* pCAN_Transmit = CAN_Pointer[IFR_Can_ID_Get(_hcan)];
	pCAN_Transmit->CAN_MultiMessage_Transmit();
}

void IFR_CAN_ClassDef::CAN_TransmitForMotor(void)
{
	uint8_t i,_Flag_[3]={0};
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	for(i=0;i<Class_Num;i++)
	{
		if(DJI_MotorClass[i]!=NULL)
		{
			switch(DJI_MotorClass[i]->Motor_Num)
			{
				case 0x201:
					_Flag_[0]++;
					MotorData[0][0]= (DJI_MotorClass[i]->output)>>8;
					MotorData[0][1]= (DJI_MotorClass[i]->output);
					break;
				case 0x205:
					_Flag_[1]++;
					MotorData[1][0]= (DJI_MotorClass[i]->output)>>8;
					MotorData[1][1]= (DJI_MotorClass[i]->output);
					break;
				case 0x209:
					_Flag_[2]++;
					MotorData[2][0]= (DJI_MotorClass[i]->output)>>8;
					MotorData[2][1]= (DJI_MotorClass[i]->output);
					break;
				case 0x202:
					_Flag_[0]++;
					MotorData[0][2]= (DJI_MotorClass[i]->output)>>8;
					MotorData[0][3]= (DJI_MotorClass[i]->output);
					break;
				case 0x206:
					_Flag_[1]++;
					MotorData[1][2]= (DJI_MotorClass[i]->output)>>8;
					MotorData[1][3]= (DJI_MotorClass[i]->output);
					break;
				case 0x20A:
					_Flag_[2]++;
					MotorData[2][2]= (DJI_MotorClass[i]->output)>>8;
					MotorData[2][3]= (DJI_MotorClass[i]->output);
					break;
				case 0x203:
					_Flag_[0]++;
					MotorData[0][4]= (DJI_MotorClass[i]->output)>>8;
					MotorData[0][5]= (DJI_MotorClass[i]->output);
					break;
				case 0x207:
					_Flag_[1]++;
					MotorData[1][4]= (DJI_MotorClass[i]->output)>>8;
					MotorData[1][5]= (DJI_MotorClass[i]->output);
					break;
				case 0x20B:
					_Flag_[2]++;
					MotorData[2][4]= (DJI_MotorClass[i]->output)>>8;
					MotorData[2][5]= (DJI_MotorClass[i]->output);
					break;
				case 0x204:
					_Flag_[0]++;
					MotorData[0][6]= (DJI_MotorClass[i]->output)>>8;
					MotorData[0][7]= (DJI_MotorClass[i]->output);
					break;
				case 0x208:
					_Flag_[1]++;
					MotorData[1][6]= (DJI_MotorClass[i]->output)>>8;
					MotorData[1][7]= (DJI_MotorClass[i]->output);
					break;
			}
		}
	}
	if(_Flag_[0] !=0)
	{
		TxMessage.StdId = 0x200;
		CAN_Transmit(&TxMessage,MotorData[0]);
	}
	if(_Flag_[1] !=0)
	{
		TxMessage.StdId = 0x1ff;
		CAN_Transmit(&TxMessage,MotorData[1]);
	}
	if(_Flag_[2] !=0)
	{
		TxMessage.StdId = 0x2ff;
		CAN_Transmit(&TxMessage,MotorData[2]);
	}
}

static uint8_t IFR_Can_ID_Get(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1) 			return 1;
	else if(hcan->Instance == CAN2) return 2;
	else return 0;
}
#endif
#endif

