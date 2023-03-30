/**
  **************************** Copyright ****************************** 
  *
  *                 (C) Copyright 2022, China, IFR机器人实验室.
  *                         All Rights Reserved
  *                              
  *    
  * FileName 	: init.cpp
  * Version		: v1.0
  * Author		: LiuHao
  * Date		: 2022-03-03
  * Description	: 

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "init.h"

/* Private variables -------------------------------------------------*/
//GPIO
IFR_GPIO_ClassDef			LED_White(GPIOC,LED_White_Pin);
IFR_GPIO_ClassDef			LED_Bule(GPIOC,LED_Blue_Pin);
IFR_GPIO_ClassDef			Key(GPIOC,KEY_Pin);
//定时器
IFR_TIM_ClassDef			TestTim;
//串口通信
IFR_UDBRx_ClassDef 		Remote_Receive;
IFR_UDBRx_ClassDef 		IMU_Receive;
IFR_UDBRx_ClassDef 		Vision_Receive;
IFR_UDBRx_ClassDef 		IFR_Remote_Receive;
IFR_UDBRx_ClassDef 		Laser_Receive;

extern osThreadId AnalysisTaskHandle;

void Test_TIM_Function(void)
{
	if(Key.Read() == GPIO_PIN_SET)		LED_Bule.High();
	if(Key.Read() == GPIO_PIN_RESET)	LED_Bule.Low();
	LED_White.Turn();
}

void All_Init(void)
{
	Arm_Motor_Init();
	Yun_Motor_Init();
	Base_Motor_Init();

	TestTim.TIM_ITStart(&htim2,Test_TIM_Function);
	
	Remote_Receive.UDB_Recevice_Init(&huart1,IFR_DJI_Remote_Analysis,&AnalysisTaskHandle);
	IMU_Receive.UDB_Recevice_Init(&huart2,IMU_Analysis,&AnalysisTaskHandle);
	Vision_Receive.UDB_Recevice_Init(&huart3,Vision_Data_Process,&AnalysisTaskHandle);
	Laser_Receive.UDB_Recevice_Init(&huart6,Laser_Analysis,&AnalysisTaskHandle);
//	IFR_Remote_Receive.UDB_Recevice_Init(&huart6,IFR_Remote_Analysis);
	Laser_Open();
	
	int i;
	for(i=0;i<3;i++)
	{
		CAN_Base.CAN_Init(&hcan1,&Robo_Base.Motor_Move[i]);
		CAN_Base.CAN_Init(&hcan1,&Robo_Base.Motor_Turn[i]);
	}
	CAN_Yun.CAN_Init(&hcan2,&Robo_YunAndArm.Motor_Pitch);
	CAN_Yun.CAN_Init(&hcan2,&Robo_YunAndArm.Motor_Launch);
	CAN_Yun.CAN_Init(&hcan2,&Robo_YunAndArm.Motor_Arm);
	CAN_Yun.CAN_Init(&hcan2,&Robo_YunAndArm.Motor_Wrist);
	
	Robo_YunAndArm.Friction_PulleyLeft.OPWM_Init(&htim3,TIM_CHANNEL_1);
	Robo_YunAndArm.Friction_PulleyRight.OPWM_Init(&htim3,TIM_CHANNEL_2);
	
	state = CHECK_STATE;
	state = STOP_STATE;
}

