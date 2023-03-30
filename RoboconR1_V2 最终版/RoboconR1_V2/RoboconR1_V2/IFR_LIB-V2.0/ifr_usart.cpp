/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_usart.cpp
  * Version		: v1.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-20
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_usart.h"

/* Private variables -------------------------------------------------*/
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
#ifdef HAL_UART_MODULE_ENABLED
IFR_UDBRx_ClassDef* UDB_Pointer[9];

void IFR_UDBRx_ClassDef::UDB_Recevice_Init(UART_HandleTypeDef *huart,void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len))
{
	_huart = huart;UDB_Pointer[IFR_Uart_ID_Get(huart)] = this;

	AnalysisFunc = UART_Analysis_Function;
	HAL_UART_RegisterRxEventCallback(huart,IFR_Uart_DoubleBuffer_Recevice_Callback);

	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	huart->ReceptionType=HAL_UART_RECEPTION_TOIDLE;
	huart->RxXferSize = IFR_UDBRX_Len_Max;
	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->DR,(uint32_t)Memory[0],(uint32_t)Memory[1],IFR_UDBRX_Len_Max);
	__USART_DMA_ENABLE(huart);
}

void IFR_UDBRx_ClassDef::UDB_Recevice_Init(UART_HandleTypeDef *huart,\
										   void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len),osThreadId *TaskHandle)
{
	_huart = huart;_TaskHandle = TaskHandle;
	UDB_Pointer[IFR_Uart_ID_Get(huart)] = this;

	AnalysisFunc = UART_Analysis_Function;
	HAL_UART_RegisterRxEventCallback(huart,IFR_FreeRTOS_Uart_DoubleBuffer_Recevice_Callback);

	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	huart->ReceptionType=HAL_UART_RECEPTION_TOIDLE;
	huart->RxXferSize = IFR_UDBRX_Len_Max;
	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->DR,(uint32_t)Memory[0],(uint32_t)Memory[1],IFR_UDBRX_Len_Max);
	__USART_DMA_ENABLE(huart);
}

void IFR_UDBRx_ClassDef::Uart_DoubleBuffer_Recevice(uint16_t len)
{
	Buffer_Num =!__DMA_GET_CURRENTMEMORY(_huart);

	__HAL_DMA_DISABLE(_huart->hdmarx);
	__DMA_CHANGE_CURRENTMEMORY(_huart);
	__HAL_DMA_ENABLE(_huart->hdmarx);

	(*AnalysisFunc)(Memory[Buffer_Num],len);
}

void IFR_Uart_DoubleBuffer_Recevice_Callback(UART_HandleTypeDef *huart,uint16_t len)
{
	IFR_UDBRx_ClassDef* pUDB_Class = UDB_Pointer[IFR_Uart_ID_Get(huart)];
	pUDB_Class->Uart_DoubleBuffer_Recevice(len);
}

void IFR_FreeRTOS_Uart_DoubleBuffer_Recevice_Callback(UART_HandleTypeDef *huart,uint16_t len)
{
	uint8_t UDB_ID = IFR_Uart_ID_Get(huart);
	IFR_UDBRx_ClassDef* pUDB_Class = UDB_Pointer[UDB_ID];
	switch(UDB_ID)
	{
		case 1:
			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart1Flag);
			break;
		case 2:
			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart2Flag);
			break;
		case 3:
			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart3Flag);
			break;
		case 4:
			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart4Flag);
			break;
		case 5:
			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart5Flag);
			break;
		case 6:
			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart6Flag);
			break;
		case 7:
			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart7Flag);
			break;
		case 8:
			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart8Flag);
			break;
	}
}

static uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) 			return 1;
	else if(huart->Instance == USART2) 	return 2;
	else if(huart->Instance == USART3) 	return 3;
	else if(huart->Instance == UART4)  	return 4;
	else if(huart->Instance == UART5)  	return 5;
	else if(huart->Instance == USART6) 	return 6;
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx) ||defined(STM32F469xx) || defined(STM32F479xx)
	else if(huart->Instance == UART7)  	return 7;
	else if(huart->Instance == UART8) 	return 8;
#endif
	else return 0;
}
#endif
#endif

