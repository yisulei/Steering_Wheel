/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: ifr_remote.c
  * Version		: v1.5
  * Author		: LiHaoyang 
  * Date		: 2022-2-27
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "remote.h"
#include "usart.h"
/* Private variables -------------------------------------------------*/
IFR_REMOTE_TypeDef IFR_REMOTE;

void IFR_Remote_Analysis(uint8_t *pData,uint8_t len) 
{     	
	if(pData[0]!=0x99) return;
	if(CRC8Calculate(pData,8)==pData[8])
	{
		IFR_REMOTE.Key1=(pData[1]>>7)&0x01;
		IFR_REMOTE.Key2=(pData[1]>>6)&0x01;
		IFR_REMOTE.Key3=(pData[1]>>5)&0x01;
		IFR_REMOTE.Key4=(pData[1]>>4)&0x01;
		
		IFR_REMOTE.Switch_Left =(pData[1]>>2)&0x03; 	
		IFR_REMOTE.Switch_Right =pData[1]&0x03; 
		
		IFR_REMOTE.Chx_Left = (pData[6]&0x03)<<8|pData[2];     
		IFR_REMOTE.Chy_Left = (pData[6]&0x0C)<<6|pData[3]; 
		IFR_REMOTE.Chx_Right =(pData[6]&0x30)<<4|pData[4];      
		IFR_REMOTE.Chy_Right =(pData[6]&0xC0)<<2|pData[5]; 
		
	}	
}


/* 计算校验值函数 */
static unsigned char CRC8Calculate(unsigned char *pBuf ,unsigned int pBufSize)
{
    unsigned char retCRCValue=0x00;
    unsigned char *pData;
    int i=0;
    unsigned char pDataBuf=0;
    pData=(unsigned char *)pBuf;

     while(pBufSize--)
     {
         pDataBuf=*pData++;
         for(i=0;i<8;i++)
         {
             if((retCRCValue^(pDataBuf))&0x01)
             {
                 retCRCValue^=0x18;
                 retCRCValue>>=1;
                 retCRCValue|=0x80;
             }
             else
             {
                   retCRCValue>>=1;
             }
            pDataBuf>>=1;
         }
     }
     return retCRCValue;
}









