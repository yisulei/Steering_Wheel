/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: laser_analysis.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-07-17
  * Description	: 

  *********************************************************************
  */

#ifndef __LASER_ANALYSIS_H_
#define __LASER_ANALYSIS_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */


/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
extern uint32_t distance;

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void Laser_Analysis(uint8_t *pData,uint8_t len);
void Laser_Open(void);
void Laser_Close(void);
uint8_t BCC(uint8_t* dat,uint16_t len);
uint16_t CRC16(uint8_t *Start_Byte, uint8_t Num_Bytes);
/* USER CODE END Prototypes */
 
#endif
