/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_remote.h
  * Version		: v1.5
  * Author		: LiHaoyang  
  * Date		: 2022-2-27
  * Description	: 

  *********************************************************************
  */

#ifndef _REMOTE_H_
#define _REMOTE_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef struct
{
	uint16_t Chx_Left;
	uint16_t Chy_Left;
	uint16_t Chx_Right;
	uint16_t Chy_Right;
	uint8_t  Switch_Left;
	uint8_t  Switch_Right;
	uint8_t  Key1;
	uint8_t  Key2;
	uint8_t  Key3;
	uint8_t  Key4;
	
}IFR_REMOTE_TypeDef;


extern IFR_REMOTE_TypeDef IFR_REMOTE;
/* USER CODE BEGIN Prototypes */
void Usart_Send(void);
void IFR_Remote_Analysis(uint8_t *pData,uint8_t len);
static unsigned char CRC8Calculate(unsigned char *pBuf ,unsigned int pBufSize);
/* USER CODE END Prototypes */

#ifdef __cplusplus
 }
#endif
 
#endif
