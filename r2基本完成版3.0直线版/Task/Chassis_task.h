/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2021, China, IFR������ʵ����.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: Chassis_task.h
  * Version		: v2.0
  * Author		: Yi Sulei 
  * Date		: 2021-12-04
  * Description	: 

  *********************************************************************
  */

#ifndef __CHASSIS_TASK_H_
#define __CHASSIS_TASK_H_



/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Chassis_behavior.h"

void ALL_Init(void);
void ChassisMove_Control(void);
void Motor_ALL_Init(void);
void TEST_PID(void);
//ypy：锁住地盘,后退别伤吸盘
void ChassisMove_back(void);
//ypy:锁住地盘
void ChassisMove_lock(void);
#endif
typedef struct UART_RX_BUFFER
{
	uint8_t* Buffer[2];
	uint8_t Buffer_Num;
	uint16_t Length_Max;
}UART_RX_BUFFER;

#define TF40BUFFER_LEN_MAX 100
#define USART3_RX_LEN_MAX 9
void Usart_Distance_Init(void);
void analysis2(uint8_t* pdata,uint8_t len);
void analysis1(uint8_t* pdata,uint8_t len);
uint8_t BCC(uint8_t* dat,uint16_t len);
void Chassis_Move(void);
void Uart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,UART_RX_BUFFER* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData));
