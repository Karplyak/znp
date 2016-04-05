/**
******************************************************************************
* @file    stm32f1xx_it.c
* @brief   Interrupt Service Routines.
******************************************************************************
*
* COPYRIGHT(c) 2016 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
typedef char byte;

char msg[] = {0xFE, 0x0A, 0x26, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x88, 0x00, 0x0A, 0x02, 0x14, 0x88, 0x33};
long impulse_counter = 0;
char buffer_it[256];
size_t howlong;
char imp_flag = 0;

byte receive_buffer_1[256];
int receive_counter_1 = 0;
int message_len;

byte send_buffer_1[256];
byte gff[253];
byte gff_len;

typedef enum {
    WAIT_SOF,
    WAIT_LEN,
    WAIT_GFF,
    WAIT_FCS
} state_type;

state_type state = WAIT_SOF;
byte len;

byte calcFCS(byte *pMsg, unsigned int len) {
    uint8_t result = 0;
    *pMsg++;
    while (len--) {
        result ^= *pMsg++;
    }
    return result;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    HAL_RCC_NMI_IRQHandler();
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    /* USER CODE BEGIN SysTick_IRQn 1 */
    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */
    //char msg[] = {0xFE, 0x01, 0x26, 0x06, 0x06, 0x27};

    /* USER CODE END TIM2_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM2_IRQn 1 */
    sprintf(buffer_it, "Impulses: %d", impulse_counter);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_UART_Transmit_IT(&huart2, buffer_it, strlen(buffer_it));
    impulse_counter = 0;
    /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    
//    switch(state) {
//        case WAIT_SOF:
//        if ((receive_buffer_1[0] = 0xFE) && (message_len = -1)) {
//            state = WAIT_LEN;
//            HAL_UART_Receive_IT(&huart1, receive_buffer_1+1, 1);
//        }
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//        break;
//
//        case WAIT_LEN:
//        gff_len = receive_buffer_1[1] + GFF_CONST;
//        state = WAIT_GFF;
//        receive_counter_1 = 1;
//        HAL_UART_Receive_IT(&huart1, receive_buffer_1+2,1);
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//        break;
//
//        case WAIT_GFF:
//        if (receive_counter_1 == gff_len - 1) {
//            HAL_UART_Receive_IT(&huart1, receive_buffer_1+receive_counter_1,1);
//            ++receive_counter_1;
//            break;
//        }
//        state = WAIT_FCS;
//        gff[0] = gff_len - GFF_CONST;
//        memcpy(gff + 1, receive_buffer_1, gff_len - 1);
//        HAL_UART_Receive_IT(&huart1, receive_buffer_1, 1);
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//        break;
//
//        case WAIT_FCS:
//        state = WAIT_SOF;
//        if (receive_buffer_1[0] == calcFCS(gff, gff_len)) {
//            // stm32 actions on zb response here
//            awesome_print(gff, gff_len);
//            printf("\n");
//            //perform_setup();
//        }
//        HAL_UART_Receive_IT(&huart1, receive_buffer_1, 1);
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//        break;
//        default:
//        break;
//    }
  if (receive_counter_1 == 0) {
    if (receive_buffer_1[0] == 0xFE) {
      HAL_UART_Receive_IT(&huart1, receive_buffer_1+(receive_counter_1++), 1);
    } else { 
      HAL_UART_IRQHandler(&huart1);
      return;
    }
  }else if (receive_counter_1 == 1) {
      message_len = receive_buffer_1[1] + GFF_CONST;
      HAL_UART_Receive_IT(&huart1, receive_buffer_1+(receive_counter_1++), 1);
  } else if (receive_counter_1 < message_len) {
      HAL_UART_Receive_IT(&huart1, receive_buffer_1+(receive_counter_1++), 1);
  } else {
    if (receive_buffer_1[message_len] == calcFCS(receive_buffer_1, message_len)) {
      printf("success\n");
    } else {
       printf("failed\n");
    }
    
  }
  
      /* USER CODE END USART1_IRQn 0 */
      HAL_UART_IRQHandler(&huart1);
      /* USER CODE BEGIN USART1_IRQn 1 */

      /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */

    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */

    /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
