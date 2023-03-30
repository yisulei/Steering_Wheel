/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_usart.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2021-10-30
  * Description	: 

  *********************************************************************
  */

#ifndef __IFR_USART_H_
#define __IFR_USART_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

//#ifndef HAL_UART_MODULE_ENABLED
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
#define BUFFER_LEN_MAX 50

//开启串口DMA通道
#define __USART_DMA_ENABLE(__HANDLE__)          ((__HANDLE__)->Instance->CR3 |=  USART_CR3_DMAR)
//获取当前数据地址
#define __DMA_GET_CURRENTMEMORY(__HANDLE__)     ((__HANDLE__)->hdmarx->Instance->CR & DMA_SxCR_CT)
//改变当前数据地址
#define __DMA_CHANGE_CURRENTMEMORY(__HANDLE__)  ((__DMA_GET_CURRENTMEMORY(__HANDLE__)!=0)\
                                                ?((__HANDLE__)->hdmarx->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT))\
                                                :((__HANDLE__)->hdmarx->Instance->CR |= (uint32_t)(DMA_SxCR_CT)))\
//获取串口ID
#define __IFR_USART_ID_GET(__HANDLE__) \
((__HANDLE__)->Instance==USART1?1:\
 (__HANDLE__)->Instance==USART2?2:\
 (__HANDLE__)->Instance==USART3?3:\
 (__HANDLE__)->Instance==UART4 ?4:\
 (__HANDLE__)->Instance==UART5 ?5:\
 (__HANDLE__)->Instance==USART6?6:0)\
 
typedef struct
{
	uint8_t  Memory[2][BUFFER_LEN_MAX];
	uint16_t Data_Length;
	void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len);
}UART_LIST_TypeDef;
					
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void IFR_UART_Init(UART_HandleTypeDef *huart,void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len));
void IFR_UART_Double_Buffer_Recevice_Callback(UART_HandleTypeDef *huart,uint16_t Remain_Len);
/* USER CODE END Prototypes */
#endif
//#endif

