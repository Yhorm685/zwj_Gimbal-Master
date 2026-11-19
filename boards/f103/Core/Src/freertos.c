/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "Application/runtime.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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

/* USER CODE END Variables */
/* Definitions for comm */
osThreadId_t commHandle;
const osThreadAttr_t comm_attributes = {
  .name = "comm",
  .stack_size = 640 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for control */
osThreadId_t controlHandle;
const osThreadAttr_t control_attributes = {
  .name = "control",
  .stack_size = 576 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for i2cDone */
osSemaphoreId_t i2cDoneHandle;
osStaticSemaphoreDef_t i2cDoneControl;
const osSemaphoreAttr_t i2cDone_attributes = {
  .name = "i2cDone",
  .cb_mem = &i2cDoneControl,
  .cb_size = sizeof(i2cDoneControl),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartControl(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  if (osKernelGetState() != osKernelReady)
  {
    Error_Handler();
  }
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of i2cDone */
  i2cDoneHandle = osSemaphoreNew(1, 0, &i2cDone_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  Gimbal_CreateQueues();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of comm */
  commHandle = osThreadNew(StartDefaultTask, NULL, &comm_attributes);

  /* creation of control */
  controlHandle = osThreadNew(StartControl, NULL, &control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (!commHandle || !controlHandle || !i2cDoneHandle) Error_Handler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  Gimbal_Comm();
  Error_Handler();
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

