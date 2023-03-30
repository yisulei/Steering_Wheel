/**
  **************************** Copyright ******************************
  *
  *                 (C) Copyright 2021, China, DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_can.c
  * Version		: v1.5
  * Author		: LiJiawei
  * Date		: 2022-02-27
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_can.h"

/* Private includes -------------------------------------------------*/


/* Private typedefs -------------------------------------------------*/


/* Private define ---------------------------------------------------*/

/* Private macro ---------------------------------------------------*/


/* Private variables -------------------------------------------------*/
#if USE_HAL_CAN_REGISTER_CALLBACKS
#ifdef HAL_CAN_MODULE_ENABLED
CAN_LIST_TypeDef CAN_List[3] = {0};
/* Private function prototypes -----------------------------------------------*/

//使能CAN接受中断
void IFR_CAN_Init(CAN_HandleTypeDef *hcan,void (*CAN_Analysis_Function)(uint8_t CAN_ID,uint8_t *Data,uint32_t stdid))
{
	uint8_t CAN_ID = __IFR_CAN_ID_GET(hcan);
	CAN_List[CAN_ID].CAN_Analysis_Function[CAN_List[CAN_ID].CAN_Analysis_Function_Num] = CAN_Analysis_Function;
	CAN_List[CAN_ID].CAN_Analysis_Function_Num++;
	
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
		
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,IFR_CAN_Recevice_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(hcan);
}

void IFR_CAN_Transmit(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
	uint32_t TxMailbox;
	uint8_t CAN_ID = __IFR_CAN_ID_GET(hcan);
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0)
	{
		HAL_CAN_AddTxMessage(hcan,pHeader,pData,&TxMailbox);
	}
	else
	{
		CAN_List[CAN_ID].pHeader[CAN_List[CAN_ID].Transmit_Flag%8]=*pHeader;
		for(int i=0;i<pHeader->DLC;i++)
			CAN_List[CAN_ID].pData[CAN_List[CAN_ID].Transmit_Flag%8][i]=pData[i];
		CAN_List[CAN_ID].Transmit_Flag++;
		if(CAN_List[CAN_ID].Transmit_Flag-CAN_List[CAN_ID].Transmit_Finish_Flag>8)
		{
			Error_Handler();
		}
		HAL_CAN_ActivateNotification(hcan,CAN_IT_TX_MAILBOX_EMPTY);
	}
}



//CAN接收中断回调函数
void IFR_CAN_Recevice_Callback(CAN_HandleTypeDef *hcan)
{
	uint8_t CAN_ID = __IFR_CAN_ID_GET(hcan);
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_List[CAN_ID].Header, CAN_List[CAN_ID].Data) == HAL_OK)
	{
		for(int i=0;i<CAN_List[CAN_ID].CAN_Analysis_Function_Num;i++)
			if(CAN_List[CAN_ID].CAN_Analysis_Function[i]!=NULL)
				CAN_List[CAN_ID].CAN_Analysis_Function[i](CAN_ID,CAN_List[CAN_ID].Data,CAN_List[CAN_ID].Header.StdId);
	}
}
//CAN发送完成中断回调函数
void IFR_CAN_Transmit_Callback(CAN_HandleTypeDef *hcan)
{
	uint8_t CAN_ID = __IFR_CAN_ID_GET(hcan);
	uint32_t TxMailbox;
	if(CAN_List[CAN_ID].Transmit_Flag>CAN_List[CAN_ID].Transmit_Finish_Flag)
	{
		HAL_CAN_AddTxMessage(hcan,&CAN_List[CAN_ID].pHeader[CAN_List[CAN_ID].Transmit_Finish_Flag%8],CAN_List[CAN_ID].pData[CAN_List[CAN_ID].Transmit_Finish_Flag%8],&TxMailbox);
		CAN_List[CAN_ID].Transmit_Finish_Flag++;
	}
	else
		HAL_CAN_DeactivateNotification(hcan,CAN_IT_TX_MAILBOX_EMPTY);
}

#endif
#endif

