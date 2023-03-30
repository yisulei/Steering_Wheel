/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_remote.c
  * Version		: v2.0
  * Author		: LiuHao Lijiawei
  * Date		: 2022-02-26
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_remote.h"

/* Private variables -------------------------------------------------*/
DJI_REMOTE_TypeDef DJI_Remote = {1024,1024,1024,1024,RC_SWITCH_UP,RC_SWITCH_UP};
KEYBOARD_TypeDef Keyboard_Data[KEY_NUMS]={0};

void IFR_DJI_Remote_Analysis(uint8_t *pData,uint8_t len)
{
	if(pData == 0)
	{
		return;
	}

	DJI_Remote.Chx_Right = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	DJI_Remote.Chy_Right = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	DJI_Remote.Chx_Left = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	DJI_Remote.Chy_Left = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	DJI_Remote.Switch_Left = ((pData[5] >> 4) & 0x000C) >> 2;
	DJI_Remote.Switch_Right = ((pData[5] >> 4) & 0x0003);

	DJI_Remote.Updata = 1;
}

void IFR_DJI_RemoteAndKeyboard_Analysis(uint8_t *pData,uint8_t len)
{
	if(pData == 0)
	{
		return;
	}

	DJI_Remote.Chx_Right = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	DJI_Remote.Chy_Right = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	DJI_Remote.Chx_Left = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	DJI_Remote.Chy_Left = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	DJI_Remote.Switch_Left = ((pData[5] >> 4) & 0x000C) >> 2;
	DJI_Remote.Switch_Right = ((pData[5] >> 4) & 0x0003);

	DJI_Remote.Mouse.X = pData[6] | (pData[7] << 8);   //!< Mouse X axis
	DJI_Remote.Mouse.Y = pData[8] | (pData[9] << 8);   //!< Mouse Y axis
	DJI_Remote.Mouse.Z = pData[10] | (pData[11] << 8); //!< Mouse Z axis
	DJI_Remote.Mouse.Press_L = pData[12]; //!< Mouse Left  Is Press ?
	DJI_Remote.Mouse.Press_R = pData[13]; //!< Mouse Right Is Press ?

	DJI_Remote.Key.V = (pData[14]|pData[15] << 8);

	DJIRemoteKey_Analysis();

	DJI_Remote.Updata = 1;
}

void IFR_DJIRemoteButton_Statu_Traversal(void)
{
		for(int keyid=0;keyid<KEY_NUMS;keyid++)
		{
			DJIRemoteButton_Statu_Get(&Keyboard_Data[keyid]);
		}
}

static void DJIRemoteButton_Statu_Get(KEYBOARD_TypeDef * Key)
{
	static int i = 0;
	if(Key->Last==1)
	{
		Key->Count++;
	}
	else
	{
		Key->Count=0;
	}

	if(Key->Count>10)
	{
		if(Key->Count<100)	//100ms
		{
			if(Key->Last==1&&Key->Value==1)
			{
				i++;
				Key->Statu=1;
				if(i == 1) Key->Short_Press = 1;
			}
			else
			{
				i = 0;
				Key->Statu=0;
			}
		}
		else
		{
			i = 0;
			if(Key->Last==1&&Key->Value==1)
			{
				Key->Statu=2;
			}
			else
			{
				Key->Statu=0;
			}
		}
	}

	Key->Last=Key->Value;
}

static void DJIRemoteKey_Analysis(void)
{
	Keyboard_Data[KEY_W].Value= DJI_Remote.Key.V&0x01;
	Keyboard_Data[KEY_S].Value=(DJI_Remote.Key.V&0x02)>>1;
	Keyboard_Data[KEY_A].Value=(DJI_Remote.Key.V&0x04)>>2;
	Keyboard_Data[KEY_D].Value=(DJI_Remote.Key.V&0x08)>>3;

	Keyboard_Data[KEY_SHIFT].Value=(DJI_Remote.Key.V&0x10)>>4;
	Keyboard_Data[KEY_CTRL].Value=(DJI_Remote.Key.V&0x20)>>5;
	Keyboard_Data[KEY_Q].Value=(DJI_Remote.Key.V&0x40)>>6;
	Keyboard_Data[KEY_E].Value=(DJI_Remote.Key.V&0x80)>>7;

	Keyboard_Data[KEY_R].Value=(DJI_Remote.Key.V&0x0100)>>8;
	Keyboard_Data[KEY_F].Value=(DJI_Remote.Key.V&0x0200)>>9;
	Keyboard_Data[KEY_G].Value=(DJI_Remote.Key.V&0x0400)>>10;
	Keyboard_Data[KEY_Z].Value=(DJI_Remote.Key.V&0x0800)>>11;
	Keyboard_Data[KEY_X].Value=(DJI_Remote.Key.V&0x1000)>>12;
	Keyboard_Data[KEY_C].Value=(DJI_Remote.Key.V&0x2000)>>13;
	Keyboard_Data[KEY_V].Value=(DJI_Remote.Key.V&0x4000)>>14;
	Keyboard_Data[KEY_B].Value=(DJI_Remote.Key.V&0x8000)>>15;
}

