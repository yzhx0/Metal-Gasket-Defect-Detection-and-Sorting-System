/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#define UART1_TX_RING_SIZE 64U
#define UART1_RX_BUF_SIZE   64U

static uint8_t txRing[UART1_TX_RING_SIZE];
static uint8_t rxIsrByte;
static uint8_t txIsrByte;
static volatile uint16_t txHead;
static volatile uint16_t txTail;
static volatile uint32_t isrErrorCount;

static StreamBufferHandle_t rxStreamHandle;
static StaticStreamBuffer_t rxStreamCb;
static uint8_t rxStreamStorage[UART1_RX_BUF_SIZE + 1U];
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */
    /* USART1 interrupt enabled with priority 5, which is at or below
       configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY so that FreeRTOS
       FromISR APIs may be called from the ISR. */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static uint16_t UART1_TxRingFreeSpace(void)
{
    if (txHead >= txTail)
    {
        return (uint16_t)(UART1_TX_RING_SIZE - 1U - (uint16_t)(txHead - txTail));
    }
    return (uint16_t)((uint16_t)(txTail - txHead) - 1U);
}

void UART1_IsrStart(void)
{
    if (rxStreamHandle == NULL)
    {
        rxStreamHandle = xStreamBufferCreateStatic(UART1_RX_BUF_SIZE + 1U,
                                                   1U,
                                                   rxStreamStorage,
                                                   &rxStreamCb);
    }
    if (rxStreamHandle != NULL)
    {
        (void)HAL_UART_Receive_IT(&huart1, &rxIsrByte, 1U);
    }
}

bool UART1_IsrSend(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    bool wasEmpty;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (UART1_TxRingFreeSpace() < length)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    wasEmpty = (txHead == txTail);
    for (index = 0U; index < length; ++index)
    {
        txRing[txHead] = data[index];
        txHead = (uint16_t)((txHead + 1U) % UART1_TX_RING_SIZE);
    }
    taskEXIT_CRITICAL();

    /* Kick off the transmitter only when it was idle.  TxCpltCallback
       chains the remaining bytes afterwards. */
    if (wasEmpty)
    {
        txIsrByte = txRing[txTail];
        (void)HAL_UART_Transmit_IT(&huart1, &txIsrByte, 1U);
    }
    return true;
}

bool UART1_IsrReadByte(uint8_t *byte, uint32_t timeoutMs)
{
    if (rxStreamHandle == NULL)
    {
        return false;
    }
    return xStreamBufferReceive(rxStreamHandle,
                                byte,
                                1U,
                                pdMS_TO_TICKS(timeoutMs)) == 1U;
}

uint32_t UART1_ConsumeErrorCount(void)
{
    uint32_t count;

    taskENTER_CRITICAL();
    count = isrErrorCount;
    isrErrorCount = 0U;
    taskEXIT_CRITICAL();

    return count;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uartHandle)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (uartHandle->Instance != USART1)
    {
        return;
    }

    if ((rxStreamHandle == NULL) ||
        (xStreamBufferSendFromISR(rxStreamHandle,
                                  &rxIsrByte,
                                  1U,
                                  &higherPriorityTaskWoken) != 1U))
    {
        ++isrErrorCount;
    }
    (void)HAL_UART_Receive_IT(uartHandle, &rxIsrByte, 1U);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance != USART1)
    {
        return;
    }

    txTail = (uint16_t)((txTail + 1U) % UART1_TX_RING_SIZE);
    if (txTail != txHead)
    {
        txIsrByte = txRing[txTail];
        (void)HAL_UART_Transmit_IT(uartHandle, &txIsrByte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance != USART1)
    {
        return;
    }

    ++isrErrorCount;
    __HAL_UART_CLEAR_PEFLAG(uartHandle);
    __HAL_UART_CLEAR_FEFLAG(uartHandle);
    __HAL_UART_CLEAR_NEFLAG(uartHandle);
    __HAL_UART_CLEAR_OREFLAG(uartHandle);
    if (rxStreamHandle != NULL)
    {
        (void)HAL_UART_Receive_IT(uartHandle, &rxIsrByte, 1U);
    }
}
/* USER CODE END 1 */

