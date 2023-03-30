/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "init.h"
#include "Robo_Task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern IFR_UDBRx_ClassDef 		Remote_Receive;
extern IFR_UDBRx_ClassDef 		IMU_Receive;
extern IFR_UDBRx_ClassDef 		Vision_Receive;
extern IFR_CAN_ClassDef				CAN_Base;
extern IFR_CAN_ClassDef 			CAN_Yun;

/* USER CODE END Variables */
osThreadId InitTaskHandle;
osThreadId AnalysisTaskHandle;
osThreadId RoboTaskHandle;
osThreadId BaseTaskHandle;
osThreadId ArmYunTaskHandle;
osThreadId AutomaticTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartInitTask(void const * argument);
void DataAnalysisTask(void const * argument);
void RoboControlTask(void const * argument);
void BaseCtrlTask(void const * argument);
void ArmYunCtrlTask(void const * argument);
void AutomaticCtrlTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of InitTask */
  osThreadDef(InitTask, StartInitTask, osPriorityIdle, 0, 128);
  InitTaskHandle = osThreadCreate(osThread(InitTask), NULL);

  /* definition and creation of AnalysisTask */
  osThreadDef(AnalysisTask, DataAnalysisTask, osPriorityNormal, 0, 128);
  AnalysisTaskHandle = osThreadCreate(osThread(AnalysisTask), NULL);

  /* definition and creation of RoboTask */
  osThreadDef(RoboTask, RoboControlTask, osPriorityHigh, 0, 128);
  RoboTaskHandle = osThreadCreate(osThread(RoboTask), NULL);

  /* definition and creation of BaseTask */
  osThreadDef(BaseTask, BaseCtrlTask, osPriorityAboveNormal, 0, 128);
  BaseTaskHandle = osThreadCreate(osThread(BaseTask), NULL);

  /* definition and creation of ArmYunTask */
  osThreadDef(ArmYunTask, ArmYunCtrlTask, osPriorityAboveNormal, 0, 128);
  ArmYunTaskHandle = osThreadCreate(osThread(ArmYunTask), NULL);

  /* definition and creation of AutomaticTask */
  osThreadDef(AutomaticTask, AutomaticCtrlTask, osPriorityHigh, 0, 128);
  AutomaticTaskHandle = osThreadCreate(osThread(AutomaticTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartInitTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInitTask */
void StartInitTask(void const * argument)
{
  /* USER CODE BEGIN StartInitTask */
	All_Init();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartInitTask */
}

/* USER CODE BEGIN Header_DataAnalysisTask */
/**
* @brief Function implementing the AnalysisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataAnalysisTask */
void DataAnalysisTask(void const * argument)
{
  /* USER CODE BEGIN DataAnalysisTask */
	osEvent event;
  /* Infinite loop */
  for(;;)
  {
		event = osSignalWait(IFR_Uart1Flag|IFR_Uart2Flag|IFR_Uart3Flag, osWaitForever);
		if(event.status==osEventSignal)
		{
			if(event.value.signals & IFR_Uart1Flag) 
				Remote_Receive.Uart_DoubleBuffer_Recevice(0);
			if(event.value.signals & IFR_Uart2Flag) 
				IMU_Receive.Uart_DoubleBuffer_Recevice(0);
			if(event.value.signals & IFR_Uart3Flag) 
				Vision_Receive.Uart_DoubleBuffer_Recevice(0);
		}
		osDelay(1);
  }
  /* USER CODE END DataAnalysisTask */
}

/* USER CODE BEGIN Header_RoboControlTask */
/**
* @brief Function implementing the RoboTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RoboControlTask */
void RoboControlTask(void const * argument)
{
  /* USER CODE BEGIN RoboControlTask */
  /* Infinite loop */
  for(;;)
  {
		Change_State();
    osDelay(1);
  }
  /* USER CODE END RoboControlTask */
}

/* USER CODE BEGIN Header_BaseCtrlTask */
/**
* @brief Function implementing the BaseTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BaseCtrlTask */
void BaseCtrlTask(void const * argument)
{
  /* USER CODE BEGIN BaseCtrlTask */
  /* Infinite loop */
  for(;;)
  {
		Base_Task(&state);
    osDelay(1);
  }
  /* USER CODE END BaseCtrlTask */
}

/* USER CODE BEGIN Header_ArmYunCtrlTask */
/**
* @brief Function implementing the ArmYunTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ArmYunCtrlTask */
void ArmYunCtrlTask(void const * argument)
{
  /* USER CODE BEGIN ArmYunCtrlTask */
  /* Infinite loop */
  for(;;)
  {
//		Yun_Task(&state);
//		Arm_Task(&state);
//		CAN_Yun.CAN_TransmitForMotor();
    osDelay(1);
  }
  /* USER CODE END ArmYunCtrlTask */
}

/* USER CODE BEGIN Header_AutomaticCtrlTask */
/**
* @brief Function implementing the AutomaticTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AutomaticCtrlTask */
void AutomaticCtrlTask(void const * argument)
{
  /* USER CODE BEGIN AutomaticCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AutomaticCtrlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
