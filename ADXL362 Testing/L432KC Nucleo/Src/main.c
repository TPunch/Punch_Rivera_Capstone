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
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ADXL362 Registers
#define DEVID_AD 		0x00
#define DEVID_MST		0x01
#define PARTID			0x02
#define REVID			0x03
#define XDATA		 	0x08
#define YDATA 			0x09
#define ZDATA 			0x0A
#define STATUS 			0x0B
#define XDATA_L 		0x0E
#define XDATA_H 		0x0F
#define YDATA_L 		0x10
#define YDATA_H 		0x11
#define ZDATA_L 		0x12
#define ZDATA_H 		0x13
#define TEMP_L 			0x14
#define TEMP_H 			0x15
#define RESET 			0x1F
#define THRESH_ACT_L 	0x20
#define THRESH_ACT_H 	0x21
#define TIME_ACT 		0x22
#define THRESH_INACT_L 	0x23
#define THRESH_INACT_H 	0x24
#define TIME_INACT_L 	0x25
#define TIME_INACT_H 	0x26
#define ACT_INACT_CTL	0x27
#define FIFO_CONTROL	0x28
#define FIFO_SAMPLES	0x29
#define INTMAP1 		0x2A
#define INTMAP2 		0x2B
#define FILTER_CTL 		0x2C
#define POWER_CTL 		0x2D
#define SELF_TEST		0x2E

#define WR_ADXL 		0x0A
#define RD_ADXL 		0x0B
#define FIFO_ADXL		0x0D
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t x8 = 0, y8 = 0, z8 = 0;
int16_t x12 = 0, y12 = 0, z12 = 0;
uint32_t drFlag;
uint8_t data_sent[6];
uint8_t data_rec[1];
uint8_t reg;
int16_t x,y,z;
float xg, yg, zg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ADXL362_ReadReg(uint8_t address);
void ADXL362_WriteReg(uint8_t address, uint8_t cmd);
void ADXL362_Init(void);
void ADXL362_GetXYZ8(uint8_t *x, uint8_t *y, uint8_t *z);
void ADXL362_GetXYZ12(int16_t *x, int16_t *y, int16_t *z);
void drFlagSet(void);
static void EXTI_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  EXTI_Init();		// Initialize External Interrupts
  ADXL362_Init();	// Initialize ADXL362
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*
	  if(drFlag == 8)
	  {
		  ADXL362_GetXYZ8(&x8, &y8, &z8);
		  printf("x = %+04d, y = %+04d, z = %+04d\n", x8,y8,z8);
		  drFlag = 0;
	  }
	  else if(drFlag == 12)
	  {
		  ADXL362_GetXYZ12(&x12, &y12, &z12);
		  printf ("x = %+05d, y = %+05d, z = %+05d\n", x12, y12, z12);
		  //pc.printf("x = %04X, y = %04X, z = %04X\n", x12, y12, z12);
		  drFlag = 0;
	  }
	*/
	/*
	  ADXL362_WriteReg(DEVID_AD, 0xAD);
	  reg = ADXL362_ReadReg(DEVID_AD);
	  x = ((data_rec[1]<<8)|data_rec[0]);
	  y = ((data_rec[3]<<8)|data_rec[2]);
	  z = ((data_rec[5]<<8)|data_rec[4]);
	*/
	  drFlag = 8;
	  ADXL362_GetXYZ8(&x8, &y8, &z8);
	  //printf("x = %+04d, y = %+04d, z = %+04d\n", x8,y8,z8);
	  drFlag = 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END SPI1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SP1_SS_GPIO_Port, SP1_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : SP1_SS_Pin */
  GPIO_InitStruct.Pin = SP1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SP1_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADXL_INT1_Pin ADXL_INT2_Pin */
  GPIO_InitStruct.Pin = ADXL_INT1_Pin|ADXL_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief ADXL362 Read Register Function
  * @param address Address of register to read
  * @retval Value of register
  */
uint8_t ADXL362_ReadReg (uint8_t address)
{
	uint8_t val;
	uint8_t ReadAddr[2];

	ReadAddr[1] = address;		// Address
	ReadAddr[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(GPIOB, SP1_SS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_Transmit(&hspi1, ReadAddr, 2, 100); 			// Transmit the read instruction and register address
	HAL_SPI_Receive(&hspi1, &val, 1, 100);					// Receive register value
	HAL_GPIO_WritePin(GPIOB, SP1_SS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave
	HAL_Delay(10);
	return val;
}

/**
  * @brief ADXL362 Write Register Function
  * @param address Address of register to write
  * @param cmd Command byte to write to register
  * @retval None
  */
void ADXL362_WriteReg (uint8_t address, uint8_t cmd)
{
	uint8_t WriteAddr[3];

	WriteAddr[2] = cmd;			// Command
	WriteAddr[1] = address;		// Address
	WriteAddr[0] = WR_ADXL;		// Write instruction

	HAL_GPIO_WritePin(SP1_SS_GPIO_Port, SP1_SS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_Transmit(&hspi1, WriteAddr, 3, 100); 						// Transmit the write instruction, address, and command
	HAL_GPIO_WritePin(SP1_SS_GPIO_Port, SP1_SS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave
}

/**
  * @brief ADXL362 Initialization Function
  * @param None
  * @retval None
  */
void ADXL362_Init(void)
{
	uint8_t reg;	// Hold register value

	// Configure ADXL_Int1
	drFlag = 0;		// Reset data ready flag

	// Disable ADXL External Interrupts
	HAL_NVIC_DisableIRQ(ADXL_INT1_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(ADXL_INT2_EXTI_IRQn);

	// Reset ADXL362 by writing 0x52(R in ASCII) to the Soft Reset Register
	HAL_Delay(100);
	ADXL362_WriteReg(RESET, 0x52);
	HAL_Delay(500);

	// Read ADXL362 registers
	//printf("\n");
	reg = ADXL362_ReadReg(DEVID_AD); 	// Read ID Register
	//printf("ID0 = 0b%X\n", reg);

	// Configure ADXL362 Filter Control Register
	reg = ADXL362_ReadReg(FILTER_CTL);   	// Read Filter Control Register
	//printf("FILTER_CTL = 0x%X\n", reg);
	// ADXL362_WriteReg(FILTER_CTL,0x51);	// Set ADXL362 to 4g range, 25Hz
	ADXL362_WriteReg(FILTER_CTL,0x11);		// Set ADXL362 to 2g range, 25Hz
	reg = ADXL362_ReadReg(FILTER_CTL);
	//printf("FILTER_CTL = 0x%X\n", reg);

	// Configure ADXL362 interrupts
	ADXL362_WriteReg(INTMAP1,0x01);		// Map Data ready Interrupt to INT1 pin
	reg = ADXL362_ReadReg(INTMAP1);
	//printf("INTMAP1 = 0x%X\n", reg);

	// Configure ADXL362 Power Control Register
	ADXL362_WriteReg(POWER_CTL,0x22);	// Set to measurement mode, ultralow noise
	reg = ADXL362_ReadReg(POWER_CTL);
	//printf("POWER_CTL = 0x%X\n", reg);

	// Begin continuous processing of ADXL362 data
	HAL_NVIC_EnableIRQ(ADXL_INT1_EXTI_IRQn);
	HAL_NVIC_EnableIRQ(ADXL_INT2_EXTI_IRQn);

}

/**
  * @brief ADXL362 Get XYZ 8bits Function
  * @param address Address of register to write
  * @param x Points to X-axis data buffer
  * @param y Points to Y-axis data buffer
  * @param z Points to Z-axis data buffer
  * @retval None
  */
void ADXL362_GetXYZ8(uint8_t *x, uint8_t *y, uint8_t *z)
{
	/*
	uint8_t xyzVal[3];
	uint8_t ReadAddr[2];

	ReadAddr[1] = XDATA;		// Address
	ReadAddr[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(SP1_SS_GPIO_Port, SP1_SS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_Transmit(&hspi1, ReadAddr, 2, 100); 						// Transmit the write instruction and address
	HAL_SPI_Receive(&hspi1, xyzVal, 3, 100);							// Receive x register value
	HAL_GPIO_WritePin(SP1_SS_GPIO_Port, SP1_SS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave

	*x = xyzVal[0];
	*y = xyzVal[1];
	*z = xyzVal[2];
	*/
	*x = ADXL362_ReadReg(XDATA);
	*y = ADXL362_ReadReg(YDATA);
	*z = ADXL362_ReadReg(ZDATA);

}

/**
  * @brief ADXL362 Get XYZ 12bits Function
  * @param x Points to X-axis data buffer
  * @param y Points to Y-axis data buffer
  * @param z Points to Z-axis data buffer
  * @retval None
  */
void ADXL362_GetXYZ12(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t xyzVal[6] = {0, 0, 0, 0, 0, 0};
    uint8_t ReadAddr[2];

	ReadAddr[1] = XDATA;		// Address
	ReadAddr[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(SP1_SS_GPIO_Port, SP1_SS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_Transmit(&hspi1, ReadAddr, 2, 100); 						// Transmit the read instruction and address
	HAL_SPI_Receive(&hspi1, xyzVal, 6, 100);				// Transmit a blank byte and receive register value;
	HAL_GPIO_WritePin(SP1_SS_GPIO_Port, SP1_SS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave

	*x = (xyzVal[1] << 8) + xyzVal[0];
	*y = (xyzVal[3] << 8) + xyzVal[2];
	*z = (xyzVal[5] << 8) + xyzVal[4];
}

/**
  * @brief External Interrupts Initialization Function
  * @param None
  * @retval None
  */
static void EXTI_Init(void)
{
	EXTI_ConfigTypeDef EXTI_ConfigStruct;
	EXTI_HandleTypeDef EXTI_HandleStruct;

	HAL_EXTI_GetHandle(&EXTI_HandleStruct, EXTI_LINE_1);
	HAL_EXTI_GetConfigLine(&EXTI_HandleStruct, &EXTI_ConfigStruct);
	HAL_EXTI_ClearConfigLine(&EXTI_HandleStruct);

	EXTI_ConfigStruct.Line = EXTI_LINE_1;
	EXTI_ConfigStruct.Mode = EXTI_MODE_INTERRUPT;
	EXTI_ConfigStruct.Trigger = EXTI_TRIGGER_RISING;
	EXTI_ConfigStruct.GPIOSel = EXTI_GPIOB;
	HAL_EXTI_RegisterCallback(&EXTI_HandleStruct, HAL_EXTI_COMMON_CB_ID, &drFlagSet);

	HAL_EXTI_SetConfigLine(&EXTI_HandleStruct, &EXTI_ConfigStruct);

	HAL_EXTI_GetHandle(&EXTI_HandleStruct, EXTI_LINE_7);
	HAL_EXTI_GetConfigLine(&EXTI_HandleStruct, &EXTI_ConfigStruct);
	HAL_EXTI_ClearConfigLine(&EXTI_HandleStruct);

	EXTI_ConfigStruct.Line = EXTI_LINE_7;
	EXTI_ConfigStruct.Mode = EXTI_MODE_INTERRUPT;
	EXTI_ConfigStruct.Trigger = EXTI_TRIGGER_RISING;
	EXTI_ConfigStruct.GPIOSel = EXTI_GPIOB;
	HAL_EXTI_RegisterCallback(&EXTI_HandleStruct, HAL_EXTI_COMMON_CB_ID, &drFlagSet);

	HAL_EXTI_SetConfigLine(&EXTI_HandleStruct, &EXTI_ConfigStruct);
}

/**
  * @brief Data Ready Flag Set Function
  * @param None
  * @retval None
  */
void drFlagSet(void) {
	// Set drFlag when ADXL_Int1 is set
	drFlag = 8;
	//drFlag = 12;
}
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
