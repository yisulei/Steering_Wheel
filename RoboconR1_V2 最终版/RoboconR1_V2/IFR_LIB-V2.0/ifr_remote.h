/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_remote.h
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-26
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_REMOTE_H_
#define __IFR_REMOTE_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

typedef struct
{
	uint16_t Chx_Left;
	uint16_t Chy_Left;
	uint16_t Chx_Right;
	uint16_t Chy_Right;
	uint8_t  Switch_Left;
	uint8_t  Switch_Right;

	struct
	{
		int16_t X;
		int16_t Y;
		int16_t Z;
		uint8_t Press_L;
		uint8_t Press_R;
	}Mouse;

 struct
 {
		uint16_t V;
 } Key;

 uint8_t Updata;

}DJI_REMOTE_TypeDef;

typedef struct
{
	uint16_t Count;
	uint8_t  Value;
	uint8_t  Last;
	uint8_t  Statu;
	uint8_t  Short_Press;
}KEYBOARD_TypeDef;

#define RC_SWITCH_UP        1
#define RC_SWITCH_MIDDLE    3
#define RC_SWITCH_DOWN      2

enum KEYBOARDID
{
	KEY_W,
	KEY_S,
	KEY_A,
	KEY_D,
	KEY_SHIFT,
	KEY_CTRL,
	KEY_Q,
	KEY_E,\
	\
	KEY_R,
	KEY_F,
	KEY_G,
	KEY_Z,
	KEY_X,
	KEY_C,
	KEY_V,
	KEY_B,
	KEY_NUMS,
};

extern DJI_REMOTE_TypeDef DJI_Remote;
extern KEYBOARD_TypeDef Keyboard_Data[KEY_NUMS];
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void IFR_DJI_Remote_Analysis(uint8_t *pData,uint8_t len);
void IFR_DJI_RemoteAndKeyboard_Analysis(uint8_t *pData,uint8_t len);
void IFR_DJIRemoteButton_Statu_Traversal(void);
void DJIRemoteButton_Statu_Get(KEYBOARD_TypeDef * Key);
void DJIRemoteKey_Analysis(void);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
 }
#endif

#endif

