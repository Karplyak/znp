/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* USER CODE BEGIN Includes */
#include <string.h>
#include "constants.h"
#define GFF_CONST 3

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void perform_setup(void);
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
typedef char byte;

void awesome_print(byte *buffer, int length) {
  for (int i = 0; i < length; i++) {
    printf("%02X", buffer[i]);
  }
}
//byte SETTINGS_RESET[] ={0xFE, 0x03, 0x26, 0x05, 0x03, 0x01, 0x03, 0x21};
byte SETTINGS_RESET[] ={0xFE, 0x03, 0x26, 0x05, 0x03, 0x01, 0x00, 0x22};
//byte SYS_RESET_REQ[] = {0xFE, 0x00, 0x21, 0x00, 0x21};
byte SYS_RESET_REQ[] = {0xFE, 0x01, 0x41, 0x00, 0x00, 0x40};
//byte TXPOWER[] = {0xFE, 0x02, 0x21, 0x0F, 0x00, 0x0F, 0x23};
//byte SETUP_1[] = {0xFE, 0x03, 0x26, 0x05, 0x87, 0x01, 0x00, 0xA6}; //Coordinator
byte SETUP_1[] = {0xFE, 0x03, 0x26, 0x05, 0x87, 0x01, 0x01, 0xA7}; //Router
byte SETUP_2[] = {0xFE, 0x04, 0x26, 0x05, 0x83, 0x02, 0x13, 0x37, 0x82};
//byte SETUP_3[] = {0xFE, 0x06, 0x26, 0x05, 0x84, 0x04, 0x00, 0x00, 0x08, 0x00, 0xAD};
byte SETUP_3[] = {0xFE, 0x12, 0x26, 0x05, 0x62, 0x10, 0x13, 0x37, 0x13, 0x37, 0x13, 0x37, 0x13, 0x37, 0x13, 0x37, 0x13, 0x37, 0x13, 0x37, 0x13, 0x37, 0x43};
//byte SETUP_4[] = {0xFE, 0x03, 0x26, 0x05, 0x63, 0x01, 0x01, 0x43};
//byte SETUP_5[] = {0xFE, 0x03, 0x26, 0x05, 0x64, 0x01, 0x01, 0x44};
byte SETUP_4[] = {0xFE, 0x15, 0x26, 0x0A, 0x05, 0x05, 0x05, 0x14, 0x88, 0x01, 0x00, 0x02, 0x26, 0x03, 0x26, 0x06, 0x04, 0x46, 0x87, 0x46, 0x83, 0x66, 0x03, 0x66, 0x06, 0xA3};
byte SETUP_START[] = {0xFE, 0x00, 0x26, 0x00, 0x26};
//byte GET_PANID[] = {0xFE, 0x01, 0x26, 0x06, 0x06, 0x27};

GPIO_PinState cts;
GPIO_PinState rts;
byte receive_buffer_1[256];
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
    while (len--) {
        result ^= *pMsg++;
    }
    return result;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    }
}
int iter = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
        switch(state) {
            case WAIT_SOF:
            if (receive_buffer_1[0] = 0xFE) {
                state = WAIT_LEN;
                HAL_UART_Receive_IT(&huart1, receive_buffer_1, 1);
            }
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
            break;

            case WAIT_LEN:
            gff_len = receive_buffer_1[0] + GFF_CONST;
            state = WAIT_GFF;
            HAL_UART_Receive_IT(&huart1, receive_buffer_1, gff_len - 1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
            break;

            case WAIT_GFF:
            state = WAIT_FCS;
            gff[0] = gff_len - GFF_CONST;
            memcpy(gff + 1, receive_buffer_1, gff_len - 1);
            HAL_UART_Receive_IT(&huart1, receive_buffer_1, 1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
            break;

            case WAIT_FCS:
            state = WAIT_SOF;
            if (receive_buffer_1[0] == calcFCS(gff, gff_len)) {
                // stm32 actions on zb response here
                printf("%d: ", iter++);
                awesome_print(gff, gff_len);
                printf("\n");
                perform_setup();
            }
            HAL_UART_Receive_IT(&huart1, receive_buffer_1, 1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
            break;
            default:
            break;
        }
    }
    if (huart == &huart2) {
        
    }
}
typedef enum {
  //S_RESET_SET,
  S_RESTART,
  S_SETUP_1,
  S_SETUP_2,
  S_SETUP_3,
  S_SETUP_4,
  S_START,
  S_START_RES_1,
  S_START_RES_2,
  NOTHING
} setup_state;
//setup_state s_state = S_RESET_SET;
setup_state s_state = S_RESTART;

void perform_setup() {
  do {
    cts = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
  } while (cts == GPIO_PIN_SET);
  switch (s_state) {
//  case S_RESET_SET:
//    HAL_UART_Transmit_IT(&huart1, SETTINGS_RESET, sizeof(SETTINGS_RESET));
//    s_state++;
//    break;
  case S_RESTART:
    HAL_UART_Transmit_IT(&huart1, SYS_RESET_REQ, sizeof(SYS_RESET_REQ));
    s_state++;
    break;
  case S_SETUP_1:
    HAL_UART_Transmit_IT(&huart1, SETUP_1, sizeof(SETUP_1));
    s_state++;
    break;
  case S_SETUP_2:
    HAL_UART_Transmit_IT(&huart1, SETUP_2, sizeof(SETUP_2));
    s_state++;
    break;
  case S_SETUP_3:
    HAL_UART_Transmit_IT(&huart1, SETUP_3, sizeof(SETUP_3));
    s_state++;
    break;
  case S_SETUP_4:
    HAL_UART_Transmit_IT(&huart1, SETUP_4, sizeof(SETUP_4));
    s_state++;
    break;
  case S_START:
    
    HAL_UART_Transmit_IT(&huart1, SETUP_START, sizeof(SETUP_START));
    s_state++;
    break;
  case S_START_RES_1:
    s_state++;
    break;
  case S_START_RES_2:
    //HAL_TIM_Base_Start_IT(&htim2);
    s_state = NOTHING;
    break;
  case NOTHING:
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    //HAL_UART_Transmit_IT(&huart2, gff, gff_len);
  default:
    break;
  }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  //RESET 30 MINUTES STAMPS
  memset(impulses_30minutes, 0, sizeof(impulses_30minutes));
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  
  //HAL_UART_Receive_IT(&huart1, receive_buffer_1, 1);
  //memcpy(send_buffer_1, SYS_RESET_REQ, sizeof(SYS_RESET_REQ));
  //HAL_UART_Transmit_IT(&huart1, send_buffer_1, sizeof(SYS_RESET_REQ));
   
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_UART_Receive_IT(&huart1, receive_buffer_1, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_UART_Transmit_IT(&huart1, SYS_RESET_REQ, sizeof(SYS_RESET_REQ));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0x1;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, FORMAT_BCD);
  
  sAlarm.AlarmTime.Hours = sTime.Hours;
  sAlarm.AlarmTime.Minutes = sTime.Minutes;
  sAlarm.AlarmTime.Seconds = sTime.Seconds + 15;
  
  HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BIN);
}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA1 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
