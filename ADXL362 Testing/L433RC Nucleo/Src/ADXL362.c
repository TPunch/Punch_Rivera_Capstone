/* Includes ------------------------------------------------------------------*/
#include "ADXL362.h"

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;
extern uint32_t drFlag;

uint8_t ADXL362_ReadReg (uint8_t address)
{
	uint8_t rxBuf[3], txBuf[3];

	txBuf[2] = 0x0;
	txBuf[1] = address;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, 3);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);	// Pull CS pin high to disable the slave

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
	uint8_t txBuf[3], rxBuf[3];

	txBuf[2] = cmd;			// Command
	txBuf[1] = address;		// Address
	txBuf[0] = WR_ADXL;		// Write instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, 3); 						// Transmit the write instruction, address, and command
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);	// Pull CS pin high to disable the slave
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
	uint8_t rxBuf[5] = {0,0,0,0,0};
	uint8_t txBuf[5] = {0,0,0,0,0};

	txBuf[1] = XDATA;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, 5);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);	// Pull CS pin high to disable the slave

	*x = rxBuf[2];
	*y = rxBuf[3];
	*z = rxBuf[4];
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
	uint8_t rxBuf[8] = {0,0,0,0,0,0,0,0};
	uint8_t txBuf[8] = {0,0,0,0,0,0,0,0};

	txBuf[1] = XDATA_L;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, 8);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);	// Pull CS pin high to disable the slave

	*x = ((int16_t)rxBuf[3] << 8) | (int16_t)rxBuf[2];
	*y = ((int16_t)rxBuf[5] << 8) | (int16_t)rxBuf[4];
	*z = ((int16_t)rxBuf[7] << 8) | (int16_t)rxBuf[6];
}

void ADXL362_GetXYZT12(int16_t *x, int16_t *y, int16_t *z, int16_t *temp)
{
	uint8_t rxBuf[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t txBuf[10] = {0,0,0,0,0,0,0,0,0,0};

	txBuf[1] = XDATA_L;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, 10);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);	// Pull CS pin high to disable the slave

	*x = ((int16_t)rxBuf[3] << 8) | (int16_t)rxBuf[2];
	*y = ((int16_t)rxBuf[5] << 8) | (int16_t)rxBuf[4];
	*z = ((int16_t)rxBuf[7] << 8) | (int16_t)rxBuf[6];
	*temp = ((int16_t)rxBuf[9] << 8) | (int16_t)rxBuf[8];
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
