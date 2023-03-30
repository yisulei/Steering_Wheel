/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_usart.h
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-20
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_USART_H_
#define __IFR_USART_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#ifdef HAL_UART_MODULE_ENABLED
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
#include "cmsis_os.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#ifndef IFR_UDBRX_Len_Max
#define IFR_UDBRX_Len_Max 50
#endif

#define __USART_DMA_ENABLE(__HANDLE__)          ((__HANDLE__)->Instance->CR3 |=  USART_CR3_DMAR)

#define __DMA_GET_CURRENTMEMORY(__HANDLE__)     ((__HANDLE__)->hdmarx->Instance->CR & DMA_SxCR_CT)

#define __DMA_CHANGE_CURRENTMEMORY(__HANDLE__)  ((__DMA_GET_CURRENTMEMORY(__HANDLE__)!=0)\
                                                ?((__HANDLE__)->hdmarx->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT))\
                                                :((__HANDLE__)->hdmarx->Instance->CR |= (uint32_t)(DMA_SxCR_CT)))
#define IFR_Uart1Flag 1<<2
#define IFR_Uart2Flag 1<<3
#define IFR_Uart3Flag 1<<4
#define IFR_Uart4Flag 1<<5
#define IFR_Uart5Flag 1<<6
#define IFR_Uart6Flag 1<<7
#define IFR_Uart7Flag 1<<8
#define IFR_Uart8Flag 1<<9
class IFR_UDBRx_ClassDef
{
	public:
		void UDB_Recevice_Init(UART_HandleTypeDef *huart,void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len));
		void UDB_Recevice_Init(UART_HandleTypeDef *huart,void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len),osThreadId *TaskHandle);
		void Uart_DoubleBuffer_Recevice(uint16_t len);
		osThreadId *_TaskHandle;
	private:
		void (*AnalysisFunc)(uint8_t* pData,uint8_t len);

		uint8_t Memory[2][IFR_UDBRX_Len_Max];
		UART_HandleTypeDef *_huart;
		uint8_t Buffer_Num;
		uint8_t Data_Length;
};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void IFR_Uart_DoubleBuffer_Recevice_Callback(UART_HandleTypeDef *huart,uint16_t len);
void IFR_FreeRTOS_Uart_DoubleBuffer_Recevice_Callback(UART_HandleTypeDef *huart,uint16_t len);
static uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif
#endif
#endif

