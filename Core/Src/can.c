/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

 // <----- Battery Messages ----->

CAN_TxHeaderTypeDef can_battery_headers[NUM_BATTERY_MSGS] = {

   /**
    * ID: 0x622
    * Data length: 7
    * Description: state of the system
    * Decision: two random 32-bit numbers
    */
    {
        .StdId = 0x0622,
        .ExtId = 0x0000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 7,
        .TransmitGlobalTime = DISABLE
    },

   /**
    * ID: 0x623
    * Data length: 6
    *
    * Description:
    *
    * - Byte 0-1: pack voltage from 0-65535V
    * - Byte 2: voltages of least charged cell (0-25.5V) [count in 100mV intervals]
    * - Byte 3: ID of cell with lowest voltage (0-255)
    * - Byte 4: voltages of least charged cell (0-25.5V) [count in 100mV intervals]
    * - Byte 5: ID of cell with lowest voltage (0-255)
    *
    * Decision: two random 32-bit numbers
    */
    {
        .StdId = 0x0623,
        .ExtId = 0x0000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 6,
        .TransmitGlobalTime = DISABLE
    },

   /**
    * ID: 0x624
    * Data length: 6
    * Description:
    *
    * - Byte 0-1: pack current (A), signed -32kA to +32kA
    * - Byte 2-3: maximum current acceptable (charge), unsigned 0 to 65kA [A]
    * - Byte 4-5: maximum current available (discharge), unsigned 0 to 65kA [A]
    *
    * Decision: two random 32-bit numbers
    */
    {
        .StdId = 0x0624,
        .ExtId = 0x0000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 6,
        .TransmitGlobalTime = DISABLE
    },

   /**
    * ID: 0x626
    * Data length: 7
    * Description:
    *
    * - Byte 0: state of charge, 0% to 100%
    *
    * Decision: random number from 0-100 is required
    */
    {
        .StdId = 0x0626,
        .ExtId = 0x0000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 7,
        .TransmitGlobalTime = DISABLE
    },

   /**
    * ID: 0x627
    * Data length: 6
    * Description:
    *
    * - Byte 0: average pack temperature, signed -127C to 127C
    * - Byte 2: sensor with min temp, signed -127C to 127C
    * - Byte 3: ID of cell with lowest temp (0-255)
    * - Byte 4: sensor with max temp, signed -127C to 127C
    * - Byte 5: ID of cell with highest temp (0-255)
    *
    * Decision: two random 32 bit numbers
    */
    {
        .StdId = 0x0627,
        .ExtId = 0x0000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 6,
        .TransmitGlobalTime = DISABLE
    }
};

 // <----- Motor Controller Messages ----->

CAN_TxHeaderTypeDef can_motor_headers[NUM_MOTOR_MSGS] = {
   /**
	 * ID: 0x401
     * Data length: 8
     * Description: Motor Velocity and Current
	 * Decision: two random 32-bit numbers
     */
	{
		.StdId = 0x0401,
		.ExtId = 0x0000,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 8,
		.TransmitGlobalTime = DISABLE
	},


   /**
    * ID: 0x501
    * Data length: 8
    * Description: status information
    * Decision: two random 32-bit numbers
    */
    {
        .StdId = 0x0501,
        .ExtId = 0x0000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    },

   /**
    * ID: 0x502
    * Data length: 8
    *
    * Description: bus voltage and current
    * Decision: two random 32-bit numbers
    */
    {
        .StdId = 0x0502,
        .ExtId = 0x0000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    },

   /**
    * ID: 0x503
    * Data length: 8
    * Description: motor and vehicle velocity
    * Decision: two random 32-bit numbers
    */
    {
        .StdId = 0x0503,
        .ExtId = 0x0000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    },

   /**
    * ID: 0x50B
    * Data length: 8
    * Description: motor temperature
    * Decision: random number from 0-100 is required
    */
    {
        .StdId = 0x050B,
        .ExtId = 0x0000,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    }
};

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
