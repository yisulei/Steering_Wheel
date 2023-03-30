/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: vision_analysis.h
  * Version		: v1.0
  * Author		: LiuHao 
  * Date		: 2022-03-04
  * Description	: 

  *********************************************************************
  */

#ifndef __VISION_ANALYSIS_H_
#define __VISION_ANALYSIS_H_

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
typedef struct
{
	uint8_t IT_look;
	uint8_t Delay_ms_vision;

	int16_t Pitch_Error;
	int16_t Yaw_Error;
	
	uint8_t Update_vision;
	
}Vision_Typedef;

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void Vision_Data_Process(uint8_t *pData,uint8_t len);

/* USER CODE END Prototypes */


#endif
