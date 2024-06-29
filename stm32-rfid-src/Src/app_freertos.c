/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RC522.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
TIM_HandleTypeDef htim1;
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
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for lightTransmit */
osThreadId_t lightTransmitHandle;
const osThreadAttr_t lightTransmit_attributes = {
  .name = "lightTransmit",
  .priority = (osPriority_t) osPriorityBelowNormal1,
  .stack_size = 128 * 4
};
/* Definitions for motorSemaphore */
osSemaphoreId_t motorSemaphoreHandle;
const osSemaphoreAttr_t motorSemaphore_attributes = {
  .name = "motorSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* Definitions for motorRun */
osThreadId_t motorRunHandle;
const osThreadAttr_t motorRun_attributes = {
  .name = "motorRun",
  .priority = (osPriority_t) osPriorityAboveNormal2,
  .stack_size = 128 * 4
};
void motorRunTask(void *arguement);
/* USER CODE END FunctionPrototypes */

void ledBlink(void *argument);
void readRFIDTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	rc522_init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* creation of motorSemaphore */
  motorSemaphoreHandle = osSemaphoreNew(1, 1, &motorSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of ledTask */
  ledTaskHandle = osThreadNew(ledBlink, NULL, &ledTask_attributes);

  /* creation of lightTransmit */
  lightTransmitHandle = osThreadNew(readRFIDTask, NULL, &lightTransmit_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  motorRunHandle = osThreadNew(motorRunTask, NULL, &motorRun_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_ledBlink */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledBlink */
void ledBlink(void *argument)
{
  /* USER CODE BEGIN ledTask */
	/*
	 * Function used for debugging. Green LED Blink
	 */
  for(;;)
  {
		GPIOC->BSRR = 1<<7;
		osDelay(100);
		GPIOC->BSRR = 1<<(7+16);
	    osDelay(100);
  }
  /* USER CODE END ledTask */
}

/* USER CODE BEGIN Header_readRFIDTask */
/**
* @brief Function implementing the readRFID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readRFIDTask */
void readRFIDTask(void *argument)
{
  /* USER CODE BEGIN lightTransmit */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END lightTransmit */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void motorRunTask(void *argument){
/*
 * Interrupt Driven Approach:
 * When interrupt for RFID comes in, take semaphore, control motors.
 * When interrupt for Motor comes in(GPIO Pin), take semaphore, control motor.
 */
	for(;;)
	{
		if(osSemaphoreAcquire(motorSemaphoreHandle, 10000) == osOK){
		  	uint8_t rfid_id[4] = {0,0,0,0};
				if(rc522_checkCard(rfid_id))
				{
					HAL_TIM_Base_Start(&htim1);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
					GPIOB->BSRR = 1<<7;
					osDelay(1000);
					GPIOB->BSRR = 1<<(7+16);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50); //Go to 0
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 50); //Go to 0
					osDelay(1000);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 75); //Go to 50
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 75); //Go to 0
					//osDelay(1000);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100); //Go to 100
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100); //Go to 100
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				}
				osDelay(1000);
		}else{
			/*
			 * LED Red
			 */
			GPIOG->BSRR = 1<<(2);
			osDelay(100);
			GPIOG->BSRR = 1<<(2+16);
			osDelay(100);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		osDelay(1);

	}
}

// External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_10) // INT Source is pin C10
    {
    //Give semaphore to motorRunTask
    	osSemaphoreRelease(motorSemaphoreHandle);
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    }
}

/*OLD PWM MOTOR CODE
 *  HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
	for(int i = 0; i < 1000000; i++){
	}
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
 *  */

/* USER CODE END Application */

