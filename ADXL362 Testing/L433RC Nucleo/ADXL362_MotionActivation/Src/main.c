/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADXL362.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T_ANG 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
int8_t x8 = 0, y8 = 0, z8 = 0;
int16_t x12 = 0, y12 = 0, z12 = 0, temp12 = 0;
volatile int16_t GarageState;
volatile uint32_t drFlag, data_counter = 0, ADXL362_AFlag;
double xyzt[400];
double xAng, yAng, zAng, tempF, xTilt, yTilt, zTilt;
double xThresh, yThresh, zThresh;
char message[200];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == ADXL362_INT1_Pin) && (ADXL362_AFlag  == 0))
	{
		// Display MCU waking up
		sprintf(message, "\r\nMCU has arisen from its slumber!\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 0xFFFF);

	}
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize ADXL362
  ADXL362_Init();

  ADXL362_GetXYZT(&x12, &y12, &z12, &temp12);		// Get X,Y,Z acceleration as ADC values for initial threshold values

  // Process XYZT values to g's (acceleration due to gravity) 1mg = 1LSB
  xyzt[0] = (0.001 * (double)x12) + X_OFFSET;				// Offsets are due to using a supply voltage of ~3.3VDC
  xyzt[1] = (0.001 * (double)y12) + Y_OFFSET;				// Offsets were adjusted primarily from the datasheet
  xyzt[2] = (0.001 * (double)z12) + Z_OFFSET;				// then manually after
  xyzt[3] = (0.065 * (double)temp12) + TEMP_OFFSET;			// 0.065 degrees C = 1LSB

  // Process g's to angles x-axis angle(Rho), y-axis angle(Phi), and z-axis angle(Theta)
  xThresh = atan2(xyzt[0], sqrt(pow(xyzt[1],2) + pow(xyzt[2], 2)));
  xThresh *= 180/M_PI;
  yThresh = atan2(xyzt[1], sqrt(pow(xyzt[0],2) + pow(xyzt[2], 2)));
  yThresh *= 180/M_PI;
  zThresh = atan2(sqrt(pow(xyzt[0], 2) + pow(xyzt[1], 2)), xyzt[2]);
  zThresh *= 180/M_PI;

  // Display & transmit current threshold angles
  sprintf(message, "\r\nxT:%+lf yT:%+lf zT:%+lf\r\n", xThresh, yThresh, zThresh);
  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 0xFFFF);
  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Check ADXL362 Awake interrupt to determine
	  ADXL362_AFlag = HAL_GPIO_ReadPin(ADXL362_INT1_GPIO_Port, ADXL362_INT1_Pin);

	  if (ADXL362_AFlag == 0){			// If the device is not moving

		  ADXL362_GetXYZT(&x12, &y12, &z12, &temp12);		// Get X,Y,Z acceleration and temperature as ADC values

		  // Process XYZT values to g's (acceleration due to gravity) 1mg = 1LSB
		  xyzt[0] = (0.001 * (double)x12) + X_OFFSET;				// Offsets are due to using a supply voltage of ~3.3VDC
		  xyzt[1] = (0.001 * (double)y12) + Y_OFFSET;				// Offsets were adjusted primarily from the datasheet
		  xyzt[2] = (0.001 * (double)z12) + Z_OFFSET;				// then manually after
		  xyzt[3] = (0.065 * (double)temp12) + TEMP_OFFSET;			// 0.065 degrees C = 1LSB

		  // Process g's to angles x-axis angle(Rho), y-axis angle(Phi), and z-axis angle(Theta)
		  xAng = atan2(xyzt[0], sqrt(pow(xyzt[1],2) + pow(xyzt[2], 2)));
		  xAng *= 180/M_PI;
		  yAng = atan2(xyzt[1], sqrt(pow(xyzt[0],2) + pow(xyzt[2], 2)));
		  yAng *= 180/M_PI;
		  zAng = atan2(sqrt(pow(xyzt[0], 2) + pow(xyzt[1], 2)), xyzt[2]);
		  zAng *= 180/M_PI;

		  // Convert degrees C to degrees F
		  tempF = (xyzt[3] * (9.0/5.0)) + 32;

		  xTilt = G_SCALER*((xThresh - 90) - (xAng - 90));
		  yTilt = G_SCALER*((yThresh - 90) - (yAng - 90));
		  zTilt = G_SCALER*((zThresh - 90) - (zAng - 90));
		  // Determine garage door state by comparing threshold and current angles
		  if((xTilt > T_ANG) || (yTilt > T_ANG) || (zTilt > T_ANG)){
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			  GarageState = 1;
		  } else{
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			  GarageState = 0;
		  }

		  // Display & transmit final garage door data before sleeping
		  sprintf(message, " X%+lf Y%+lf Z%+lf T%+lf S%d A%+lf \r\n", xAng, yAng, zAng, tempF, GarageState, zTilt);
		  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 0xFFFF);
		  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);

		  // Display MCU going to sleep
		  sprintf(message, "ADXL362 is sleeping. MCU is going night night!\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 0xFFFF);
		  HAL_Delay(200);
		  HAL_SuspendTick();
		  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		  HAL_ResumeTick();
	  } else{
		  ADXL362_GetXYZT(&x12, &y12, &z12, &temp12);		// Get X,Y,Z acceleration and temperature as ADC values
		  data_counter += 4;				// Increment data_counter
		  if(data_counter == 396){			// Calculate every 100th dataset and display
			  // Process XYZT values to g's (acceleration due to gravity) 1mg = 1LSB
			  xyzt[data_counter] = (0.001 * (double)x12) + X_OFFSET;				// Offsets are due to using a supply voltage of ~3.3VDC
			  xyzt[data_counter+1] = (0.001 * (double)y12) + Y_OFFSET;				// Offsets were adjusted primarily from the datasheet
			  xyzt[data_counter+2] = (0.001 * (double)z12) + Z_OFFSET;				// then manually after
			  xyzt[data_counter+3] = (0.065 * (double)temp12) + TEMP_OFFSET;		// 0.065 degrees C = 1LSB

			  // Process g's to angles x-axis angle(Rho), y-axis angle(Phi), and z-axis angle(Theta)
			  xAng = atan2(xyzt[data_counter], sqrt(pow(xyzt[data_counter+1],2) + pow(xyzt[data_counter+2], 2)));
			  xAng *= 180/M_PI;
			  yAng = atan2(xyzt[data_counter+1], sqrt(pow(xyzt[data_counter],2) + pow(xyzt[data_counter+2], 2)));
			  yAng *= 180/M_PI;
			  zAng = atan2(sqrt(pow(xyzt[data_counter], 2) + pow(xyzt[data_counter+1], 2)), xyzt[data_counter+2]);
			  zAng *= 180/M_PI;
			  //zAng -= 90;

			  // Convert degrees C to degrees F
			  tempF = (xyzt[data_counter+3] * (9.0/5.0)) + 32;

			  if((xTilt > T_ANG) || (yTilt > T_ANG) || (zTilt > T_ANG)){
				  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				  GarageState = 1;
			  } else{
				  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				  GarageState = 0;
			  }

			  xTilt = G_SCALER*((xThresh - 90) - (xAng - 90));
			  yTilt = G_SCALER*((yThresh - 90) - (yAng - 90));
			  zTilt = G_SCALER*((zThresh - 90) - (zAng - 90));


			  // Display & transmit garage door data
			  sprintf(message, " X%+lf Y%+lf Z%+lf T%+lf S%d A%+lf \r\n", xAng, yAng, zAng, tempF, GarageState, zTilt);
			  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 0xFFFF);
			  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xFFFF);
			  data_counter = 0;			// Reset data_counter
		  }
	  }
	  HAL_Delay(2);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADXL362_CS_Pin */
  GPIO_InitStruct.Pin = ADXL362_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ADXL362_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADXL362_INT1_Pin */
  GPIO_InitStruct.Pin = ADXL362_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADXL362_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
