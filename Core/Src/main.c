/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include <string.h>   // 新增：串口指令解析必需
#include <stdio.h>    // 新增：sprintf必需
#include <stdlib.h>   // 新增：atoi必需

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// 自动调速参数
uint16_t pwm_duty;                // 唯一定义，修复重复定义错误
uint32_t last_update_time;
uint8_t speed_phase;
const uint16_t duty_map[4] = {200, 500, 800, 500};
const uint32_t phase_interval = 10;
const uint32_t pause_time = 1200;
uint8_t is_pausing;
uint32_t pause_start_time;

// 串口参数
#define UART_BUF_SIZE 64
uint8_t uart_buf[UART_BUF_SIZE];
uint16_t uart_buf_head = 0;
uint16_t uart_buf_tail = 0;
uint8_t uart_recv_data;

// 控制模式：0=自动调速 1=串口手动控制
uint8_t control_mode = 0;
uint8_t motor_state = 1; // 默认启动
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void update_pwm_duty(void);
uint8_t uart_check_cmd(char *cmd_buf, uint8_t max_len);
uint8_t uart_read_byte(void);
void parse_motor_cmd(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 自动调速函数
void update_pwm_duty(void)
{
  if(is_pausing)
  {
    if(HAL_GetTick() - pause_start_time >= pause_time)
    {
      is_pausing = 0;
      speed_phase++;
      if(speed_phase >= 4) speed_phase = 0;
    }
    return;
  }

  switch(speed_phase)
  {
    case 0: pwm_duty += 2; if(pwm_duty >= duty_map[1]){is_pausing=1; pause_start_time=HAL_GetTick();} break;
    case 1: pwm_duty += 2; if(pwm_duty >= duty_map[2]){is_pausing=1; pause_start_time=HAL_GetTick();} break;
    case 2: pwm_duty -= 2; if(pwm_duty <= duty_map[3]){is_pausing=1; pause_start_time=HAL_GetTick();} break;
    case 3: pwm_duty -= 2; if(pwm_duty <= duty_map[0]){is_pausing=1; pause_start_time=HAL_GetTick();} break;
  }
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_duty);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  // 初始化参数
  pwm_duty = duty_map[0];
  last_update_time = 0;
  speed_phase = 0;
  is_pausing = 0;
  pause_start_time = 0;

  // 启动PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // 正转：PB0=1，PB1=0（修复方向引脚错误）
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  // 开启串口中断
  HAL_UART_Receive_IT(&huart1, &uart_recv_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. 串口指令处理（优先级最高）
    char cmd_buf[32] = {0};
    if(uart_check_cmd(cmd_buf, 32))
    {
      parse_motor_cmd(cmd_buf);
    }

    // 2. 自动调速模式（仅手动模式关闭时生效）
    if(control_mode == 0 && motor_state == 1)
    {
      if (HAL_GetTick() - last_update_time > phase_interval)
      {
        last_update_time = HAL_GetTick();
        update_pwm_duty();
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_USART1_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, Green_Pin|Yellow_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Green_Pin|Red_Pin|Yellow_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
// 串口中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    uint16_t next_head = (uart_buf_head + 1) % UART_BUF_SIZE;
    if(next_head != uart_buf_tail)
    {
      uart_buf[uart_buf_head] = uart_recv_data;
      uart_buf_head = next_head;
    }
    HAL_UART_Receive_IT(&huart1, &uart_recv_data, 1);
  }
}

uint8_t uart_read_byte(void)
{
  if(uart_buf_head == uart_buf_tail) return 0;
  uint8_t data = uart_buf[uart_buf_tail];
  uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
  return data;
}

uint8_t uart_check_cmd(char *cmd_buf, uint8_t max_len)
{
  static uint8_t cmd_idx = 0;
  while(uart_buf_head != uart_buf_tail && cmd_idx < max_len - 1)
  {
    uint8_t data = uart_read_byte();
    if(data == '\n' || data == '\r')
    {
      cmd_buf[cmd_idx] = '\0';
      cmd_idx = 0;
      return 1;
    }
    cmd_buf[cmd_idx++] = data;
  }
  return 0;
}

// 指令解析（新增自动/手动切换）
void parse_motor_cmd(char *cmd)
{
  char *token = strtok(cmd, " ");
  if(token == NULL){HAL_UART_Transmit(&huart1, (uint8_t*)"ERROR: Empty cmd\r\n", 17, 100); return;}

  if(strcmp(token, "AUTO") == 0)    // 切换自动调速
  {
    control_mode = 0; motor_state = 1;
    HAL_UART_Transmit(&huart1, (uint8_t*)"Mode: AUTO\r\n", 11, 100);
  }
  else if(strcmp(token, "MANUAL") == 0) // 切换手动控制
  {
    control_mode = 1;
    HAL_UART_Transmit(&huart1, (uint8_t*)"Mode: MANUAL\r\n", 13, 100);
  }
  else if(strcmp(token, "ON") == 0)
  {
    motor_state = 1;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_duty);
    HAL_UART_Transmit(&huart1, (uint8_t*)"Motor ON\r\n", 9, 100);
  }
  else if(strcmp(token, "OFF") == 0)
  {
    motor_state = 0;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_UART_Transmit(&huart1, (uint8_t*)"Motor OFF\r\n", 10, 100);
  }
  else if(strcmp(token, "SPEED") == 0)
  {
    token = strtok(NULL, " ");
    if(token == NULL){HAL_UART_Transmit(&huart1, (uint8_t*)"ERROR: No speed\r\n", 17, 100); return;}
    int speed = atoi(token);
    if(speed<0||speed>100){HAL_UART_Transmit(&huart1, (uint8_t*)"ERROR: 0-100\r\n", 13, 100); return;}
    pwm_duty = speed * 999 / 100;
    if(motor_state) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_duty);
    char buf[20]; sprintf(buf, "Speed: %d%%\r\n", speed);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
  }
  else
  {
    HAL_UART_Transmit(&huart1, (uint8_t*)"ERROR: Unknown\r\n", 15, 100);
  }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1){}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif /* USE_FULL_ASSERT */
