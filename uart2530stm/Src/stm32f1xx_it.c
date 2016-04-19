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
#include "constants.h"

/* USER CODE BEGIN 0 */
#define INPUTS 4

char msg[] = {0xFE, 0x0A, 0x26, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x88, 0x00, 0x0A, 0x02, 0x14, 0x88, 0x33};

char buffer_it[256];
size_t howlong;

char state_tim2 = 0;
//char state_tim3 = 0;
//char state_ioint = 0;
//long impulse_counter = 0;

char bounce_counter[] = {0, 0, 0, 0};
unsigned long impulse_counter[] = {0, 0, 0, 0};
char counter_tim3[] = {0, 0, 0, 0};
char state_ioint[] = {0, 0, 0, 0};
GPIO_PinState pins[INPUTS];

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
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
    
    switch (state_tim2) {
        case 0:
        state_tim2 = 1;
        break;
        case 1:
        for (int i = 0; i < INPUTS; i++) {
          printf("Impulses %d: %d\n", i, impulse_counter[i]);
          impulse_counter[i] = 0;
        }
        break;
    }
  
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  pins[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
  pins[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
  pins[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
  pins[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);

  for (int i = 0; i < INPUTS; i++) {
      if (state_ioint[i] == 1) {
          if (counter_tim3[i] < 10) {
              counter_tim3[i]++;
          } else {
              counter_tim3[i] = 0;
              if (pins[i] == GPIO_PIN_SET) {
                  state_ioint[i] = 2;
              } else {
                  state_ioint[i] = 0;
              }
          }
      }
  }
    // switch (state_tim3) {
    //     case 0:
    //     state_tim3 = 1;
    //     break;
    //     case 1:
    //     //printf("test\n");
    //     if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) {
    //         state_ioint = 2;
    //         HAL_TIM_Base_Stop(&htim3);
    //     } else {
    //         state_ioint = 0;
    //         HAL_TIM_Base_Stop(&htim3);
    //     }
    //     break;
    // }


  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

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

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
    pins[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    pins[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    pins[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
    pins[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);

    for (int i = 0; i < INPUTS; i++) {
        switch (state_ioint[i]) {
            case 0:
            //HAL_TIM_Base_Start_IT(&htim3);
            if (pins[i] == GPIO_PIN_SET) {
                state_ioint[i] = 1;
            }
            break;
            case 1:
            break;
            case 2:
            impulse_counter[i]++;
            state_ioint[i] = 0;
            break;
        }
    }
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

    //HAL_TIM_Base_Start(&htim3);
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
