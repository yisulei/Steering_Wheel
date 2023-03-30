/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_usart.c
  * Version		: v1.3
  * Author		: LiJiawei
  * Date		: 2021-10-30
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_usart.h"

/* Private variables -------------------------------------------------*/
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
#ifdef HAL_UART_MODULE_ENABLED
UART_LIST_TypeDef UART_List[9];

/**
  * 函数名称：IFR_UART_Init
  * 函数功能：使能串口接收中断，并将数据转发给相应的数据处理函数
  * 入口参数：串口编号、串口数据处理函数
  * 出口参数：数据处理函数（串口编号、数据）
  * 作者：Li Jiawei
  * 修改日期：2021-10-27
  */
void IFR_UART_Init(UART_HandleTypeDef *huart, void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len))
{
	uint8_t UART_ID = __IFR_USART_ID_GET(huart);//获取串口id
	UART_List[UART_ID].Data_Length = BUFFER_LEN_MAX;//赋值数据长度
	UART_List[UART_ID].UART_Analysis_Function= UART_Analysis_Function;//给串口数据处理函数传参
	
	HAL_UART_RegisterRxEventCallback(huart,IFR_UART_Double_Buffer_Recevice_Callback);//指定中断回调函数
	
    //串口初始化流程
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	huart->ReceptionType=HAL_UART_RECEPTION_TOIDLE;
	huart->RxXferSize = BUFFER_LEN_MAX;
	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->DR,
		(uint32_t)UART_List[UART_ID].Memory[0],(uint32_t)UART_List[UART_ID].Memory[1],BUFFER_LEN_MAX);
	__USART_DMA_ENABLE(huart);
}


/**
  * 函数名称：IFR_UART_Init
  * 函数功能：串口接收中断回调函数
  * 入口参数：串口编号
  * 出口参数：无
  * 作者：Li Jiawei
  * 修改日期：2021-10-27
  */
void IFR_UART_Double_Buffer_Recevice_Callback(UART_HandleTypeDef *huart,uint16_t Remain_Len)
{
	uint8_t UART_ID = __IFR_USART_ID_GET(huart);	
	uint8_t Buffer_Num =!__DMA_GET_CURRENTMEMORY(huart);	
	uint8_t Data_Length = UART_List[UART_ID].Data_Length -Remain_Len;
	
	__HAL_DMA_DISABLE(huart->hdmarx);
	__DMA_CHANGE_CURRENTMEMORY(huart);
	__HAL_DMA_ENABLE(huart->hdmarx);

	if(UART_List[UART_ID].UART_Analysis_Function != NULL) 
		UART_List[UART_ID].UART_Analysis_Function(UART_List[UART_ID].Memory[Buffer_Num],Data_Length);
}


#endif
#endif

