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
#include "string.h"

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
#define SOFT_RESET 		0x1F
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
int16_t x12 = 0, y12 = 0, z12 = 0;
volatile uint32_t drFlag, SPI2XxFlag;
float xg, yg, zg;
char message[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint8_t ADXL362_ReadReg(uint8_t address);
void ADXL362_WriteReg(uint8_t address, uint8_t cmd);
void ADXL362_Init(void);
void ADXL362_GetXYZ8(int8_t *x, int8_t *y, int8_t *z);
void ADXL362_GetXYZ12(int16_t *x, int16_t *y, int16_t *z);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance==SPI2)
	{
		SPI2XxFlag = 1;
		//HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave
		//HAL_Delay(10);
	}
}
*/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance==SPI2)
	{
		SPI2XxFlag = 1;
		//HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave
		//HAL_Delay(10);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == ADXL362_INT1_Pin)
	{
		drFlag = 8;
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
  ADXL362_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(SPI2XxFlag == 1){
		  if(drFlag == 8)
		  {
			  ADXL362_GetXYZ8(&x8, &y8, &z8);
			  sprintf(message, "%+04d %+04d %+04d\r\n", x8,y8,z8);
			  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 0xFFFF);
			  drFlag = 0;
		  }

		  else if(drFlag == 12)
		  {
			  ADXL362_GetXYZ12(&x12, &y12, &z12);
			  sprintf(message, "%+05d %+05d %+05d\r\n", x12, y12, z12);
			  //sprintf(message, "%04X, %04X, %04X\r\n", x12, y12, z12);
			  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 0xFFFF);
			  drFlag = 0;
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
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ADXL362_INT1_Pin ADXL362_INT2_Pin */
  GPIO_InitStruct.Pin = ADXL362_INT1_Pin|ADXL362_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
uint8_t ADXL362_ReadReg (uint8_t address)
{
	//HAL_NVIC_DisableIRQ(ADXL362_INT1_EXTI_IRQn);
	//HAL_NVIC_DisableIRQ(ADXL362_INT2_EXTI_IRQn);

	uint8_t rxBuf[3], txBuf[3];

	txBuf[2] = 0x0;
	txBuf[1] = address;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	SPI2XxFlag = 0;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, 3);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);	// Pull CS pin high to disable the slave
	//HAL_NVIC_EnableIRQ(ADXL362_INT1_EXTI_IRQn);
	//HAL_NVIC_EnableIRQ(ADXL362_INT2_EXTI_IRQn);

	return rxBuf[2];
}

/**
  * @brief ADXL362 Write Register Function
  * @param address Address of register to write
  * @param cmd Command byte to write to register
  * @retval None
  */
void ADXL362_WriteReg (uint8_t address, uint8_t cmd)
{
	//HAL_NVIC_DisableIRQ(ADXL362_INT1_EXTI_IRQn);
	//HAL_NVIC_DisableIRQ(ADXL362_INT2_EXTI_IRQn);

	uint8_t txBuf[3], rxBuf[3];

	txBuf[2] = cmd;			// Command
	txBuf[1] = address;		// Address
	txBuf[0] = WR_ADXL;		// Write instruction

	SPI2XxFlag = 0;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, 3); 						// Transmit the write instruction, address, and command
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);	// Pull CS pin high to disable the slave

	//HAL_NVIC_EnableIRQ(ADXL362_INT1_EXTI_IRQn);
	//HAL_NVIC_EnableIRQ(ADXL362_INT2_EXTI_IRQn);
}

/**
  * @brief ADXL362 Initialization Function
  * @param None
  * @retval None
  */
void ADXL362_Init(void)
{
	uint8_t reg = 0;	// Hold register value
	char msg[50];	// Message to print

	// Configure ADXL_Int1
	drFlag = 0;		// Reset data ready flag
	SPI2XxFlag = 1;	// Reset SPI transmission flag

	// Disable ADXL External Interrupts
	HAL_NVIC_DisableIRQ(ADXL362_INT1_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(ADXL362_INT2_EXTI_IRQn);

	// Reset ADXL362 by writing 0x52(R in ASCII) to the Soft Reset Register
	sprintf(msg, "Initiating ADXL362!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	ADXL362_WriteReg(SOFT_RESET, 0x52);
	HAL_Delay(1);

	// Read ADXL362 registers
	reg = ADXL362_ReadReg(DEVID_AD); 	// Read ID Register
	sprintf(msg, "ID0 = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	// Configure ADXL362 Filter Control Register
	reg = ADXL362_ReadReg(FILTER_CTL);   	// Read Filter Control Register
	sprintf(msg, "FILTER_CTL = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	// ADXL362_WriteReg(FILTER_CTL,0x51);	// Set ADXL362 to 4g range, 25Hz
	ADXL362_WriteReg(FILTER_CTL,0x11);		// Set ADXL362 to 2g range, 25Hz
	reg = ADXL362_ReadReg(FILTER_CTL);
	sprintf(msg, "FILTER_CTL = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	// Configure ADXL362 interrupts
	ADXL362_WriteReg(INTMAP1,0x01);		// Map Data ready Interrupt to INT1 pin
	reg = ADXL362_ReadReg(INTMAP1);
	sprintf(msg, "INTMAP1 = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	ADXL362_WriteReg(INTMAP2,0x40);		// Map Awake Interrupt to INT2 pin
	reg = ADXL362_ReadReg(INTMAP2);
	sprintf(msg, "INTMAP2 = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	// Configure ADXL362 Power Control Register
	ADXL362_WriteReg(POWER_CTL,0x22);	// Set to measurement mode, ultralow noise
	reg = ADXL362_ReadReg(POWER_CTL);
	sprintf(msg, "POWER_CTL = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	// Begin continuous processing of ADXL362 data
	HAL_NVIC_EnableIRQ(ADXL362_INT1_EXTI_IRQn);
	HAL_NVIC_EnableIRQ(ADXL362_INT2_EXTI_IRQn);

}

/**
  * @brief ADXL362 Get XYZ 8bits Function
  * @param address Address of register to write
  * @param x Points to X-axis data buffer
  * @param y Points to Y-axis data buffer
  * @param z Points to Z-axis data buffer
  * @retval None
  */
void ADXL362_GetXYZ8(int8_t *x, int8_t *y, int8_t *z)
{
	//HAL_NVIC_DisableIRQ(ADXL362_INT1_EXTI_IRQn);
	//HAL_NVIC_DisableIRQ(ADXL362_INT2_EXTI_IRQn);

	uint8_t xyzVal[5] = {0,0,0,0,0};
	uint8_t txBuf[5] = {0,0,0,0,0};

	txBuf[1] = XDATA;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	SPI2XxFlag = 0;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, xyzVal, 5);

	//HAL_NVIC_EnableIRQ(ADXL362_INT1_EXTI_IRQn);
	//HAL_NVIC_EnableIRQ(ADXL362_INT2_EXTI_IRQn);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);	// Pull CS pin high to disable the slave
	*x = xyzVal[2];
	*y = xyzVal[3];
	*z = xyzVal[4];
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
	uint8_t xyzVal[8] = {0,0,0,0,0,0,0,0};
	uint8_t txBuf[8] = {0,0,0,0,0,0,0,0};

	txBuf[1] = XDATA_L;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	SPI2XxFlag = 0;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, xyzVal, 8);
	//HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave

	*x = ((uint16_t)xyzVal[3] << 8) + xyzVal[2];
	*y = ((uint16_t)xyzVal[5] << 8) + xyzVal[4];
	*z = ((uint16_t)xyzVal[7] << 8) + xyzVal[6];
	//*temp = ((uint16_t)xyzVal[9] << 8) + xyzVal[8];
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
