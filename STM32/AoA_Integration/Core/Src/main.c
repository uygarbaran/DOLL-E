/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx[2];
//const uint8_t ack = 32; // ack in our case is a space (32)

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   // do nothing here
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int16_t angle = ((int16_t *)rx)[0];
//	if (angle >= 90 && angle <= 100)
	if (angle >= 40 && angle <= 50)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	// HAL_UART_Transmit_IT(&huart2, rx, 2); // Send it over to monitor (ST-LINK)
//	HAL_UART_Transmit_IT(&huart1, &ack, 1);
//	HAL_UART_Transmit_IT(&huart1, &rx, 1);
	HAL_UART_Receive_IT(&huart1, rx, 2); // UYGAR_LOG: call the interrupt function to keep the interrupt going on call back
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, rx, 2); // UYGAR_LOG: call the interrupt function
  //HAL_UART_Receive_IT(&huart1, &rx, 1); // UYGAR_LOG: call the interrupt function
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t X = 0;
//  char MSG[100];
//  const uint8_t *rx_fail = "Rx failed\n\r";
////  const char *rx_success = "Rx succeeded\n\r";
//  uint8_t Rx_data[100];
//  Rx_data[0] = '\0';
//  uint8_t tx = 0;

  //  HAL_OK       = 0x00U,
//  HAL_ERROR    = 0x01U,
//  HAL_BUSY     = 0x02U,
//  HAL_TIMEOUT  = 0x03U
//  HAL_StatusTypeDef HAL_stat = HAL_ERROR;
//  HAL_StatusTypeDef send_stat = HAL_ERROR;
  while (1)
  {
	// printf("HEY WORLD\n");
     //sprintf(MSG, "X = %d\r\n");
//	  memcpy(send, RX_);
//	  tx = 'a';
//      send_stat = HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//      HAL_Delay(1);
	  //HAL_stat = HAL_UART_Receive(&huart2, &rx, 1, HAL_MAX_DELAY);

//	  enterToNull(Rx_data, 100);

//	  if (HAL_stat != HAL_OK)
//	  {
//		  HAL_UART_Transmit(&huart2, rx_fail, strlen(rx_fail), 100);
//	  }
//	  else
//	  {
//		  HAL_UART_Transmit(&huart2, Rx_data, 1, 100);
//	  }

	  // Change this to whether you wanna check for receive or transmit
//	  HAL_StatusTypeDef stat_to_check = HAL_stat;
//	  if (stat_to_check == HAL_ERROR)
//	  {
////		  tx = 'e';
////		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
//	  else if (stat_to_check == HAL_TIMEOUT)
//	  {
////		  tx = 't';
////		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
//	  else if (stat_to_check ==HAL_OK)
//	  {
////		  tx = 'o';
////		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  HAL_Delay(1000);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//
//		  if (Rx_data[0] == 'a')
//		  {
//			  HAL_Delay(1000);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			  HAL_Delay(100);
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  }
//	  }
//	  else if (stat_to_check == HAL_BUSY)
//	  {
////		  tx = 'b';
////		  		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  HAL_Delay(1000);
//		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		HAL_Delay(1000);
//		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  				  		  HAL_Delay(1000);
//		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
//	  else
//	  {
////		  tx = '?';
////		  HAL_UART_Transmit(&huart2, &tx, sizeof(tx), 100);
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		  HAL_Delay(1000);
//		  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  		HAL_Delay(1000);
//		  		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  				  		  HAL_Delay(1000);
//		  		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  				  	HAL_Delay(1000);
//		  		  				  			  		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  		  				  			  		  				  		  HAL_Delay(1000);
//		  		  				  			  		  				  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
    //HAL_UART_Transmit(&huart2, Rx_data, sizeof(Rx_data), 100);
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	HAL_Delay(2000);
	//X++;
//	send[0]++;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
