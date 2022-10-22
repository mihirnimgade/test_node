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

#include "xorshift.h"
#include "can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

union FloatBytes {
	float float_value;
	uint8_t bytes[4];
} FloatBytes;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DATA_LENGTH 			8
#define KERNEL_LED_DELAY 		1000

#define BATTERY_TX_DELAY 		1000
#define MOTOR_TX_DELAY 			1000

#define MAX_CAN_BATT_TX_DELAY   200
#define MAX_CAN_MOTOR_TX_DELAY  200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId_t sendBatteryMsgHandle;
osThreadId_t sendMotorMsgHandle;
osThreadId_t kernelLEDHandle;

const osThreadAttr_t sendBatteryMsgAttr = {
    .name = "sendBatteryCANMsg",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 128 * 4
};

const osThreadAttr_t sendMotorMsgAttr = {
    .name = "sendMotorCANMsg",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 128 * 4
};

const osThreadAttr_t kernelLEDAttr = {
    .name = "kernelLED",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 4
};

union FloatBytes motor_temp;

HAL_StatusTypeDef status;

uint8_t error;
uint32_t can_mailbox;
uint32_t free_level;

uint16_t changing_soc = 0;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void sendBatteryMsg(void *argument);
void sendMotorMsg(void *argument);
void kernelLEDTask(void *argument);

/* USER CODE END FunctionPrototypes */

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

    /* USER CODE BEGIN RTOS_THREADS */

    sendBatteryMsgHandle = osThreadNew(sendBatteryMsg, NULL, &sendBatteryMsgAttr);
    sendMotorMsgHandle = osThreadNew(sendMotorMsg, NULL, &sendMotorMsgAttr);
    // kernelLEDHandle = osThreadNew(kernelLEDTask, NULL, &kernelLEDAttr);

    /* USER CODE END RTOS_THREADS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

__NO_RETURN void kernelLEDTask (void *argument) {

    while (1) {
        osKernelState_t kernel_status = osKernelGetState();

        if (kernel_status == osKernelRunning) {
            HAL_GPIO_TogglePin(KERNEL_LED_GPIO_Port, KERNEL_LED_Pin);
        }

        osDelay(KERNEL_LED_DELAY);
    }
}

__NO_RETURN void sendMotorMsg(void *argument) {
    CAN_TxHeaderTypeDef rand_header;

    uint8_t rand_data[8] = {0};
    uint32_t rand_index = 0;
    uint16_t rand_delay = MAX_CAN_MOTOR_TX_DELAY;

    while (1) {
        rand_index = rand(NUM_MOTOR_MSGS);
        rand_header = can_motor_headers[rand_index];

        rand_array(&rand_data[0], 8);

        free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);

        status = HAL_CAN_AddTxMessage(&hcan, &rand_header, rand_data, &can_mailbox);

        if (status != HAL_OK) {
            error = 1;
        } else {
            HAL_GPIO_TogglePin(KERNEL_LED_GPIO_Port, KERNEL_LED_Pin);
        }

        rand_delay = rand(MAX_CAN_MOTOR_TX_DELAY);
        osDelay(rand_delay);
    }
}

__NO_RETURN void sendBatteryMsg(void *argument) {
    CAN_TxHeaderTypeDef rand_header;

    uint8_t rand_data[8] = {0};
    uint32_t rand_index = 0;
    uint16_t rand_delay = MAX_CAN_BATT_TX_DELAY;

    uint8_t rand_soc = 0;

    while (1) {
        rand_index = rand(NUM_BATTERY_MSGS);
        rand_header = can_battery_headers[rand_index];

        if (rand_header.StdId == 0x626) {
            rand_soc = rand(100);
            rand_data[0] = rand_soc;
        } else {
            rand_array(&rand_data[0], 8);
        }

        free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);

        status = HAL_CAN_AddTxMessage(&hcan, &rand_header, rand_data, &can_mailbox);

        if (status != HAL_OK) {
            error = 1;
        } else {
            HAL_GPIO_TogglePin(KERNEL_LED_GPIO_Port, KERNEL_LED_Pin);
        }

        rand_delay = rand(MAX_CAN_BATT_TX_DELAY);
        osDelay(rand_delay);
    }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
