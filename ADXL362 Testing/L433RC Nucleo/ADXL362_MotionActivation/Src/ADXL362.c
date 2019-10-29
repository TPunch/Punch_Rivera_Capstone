/* Includes ------------------------------------------------------------------*/
#include "ADXL362.h"

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;
extern uint32_t ADXL362_AFlag;

uint8_t ADXL362_ReadReg (uint8_t address)
{
	uint8_t rxBuf[3], txBuf[3];

	txBuf[2] = 0x0;			// Dummy byte for reading reg
	txBuf[1] = address;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, sizeof(txBuf));			// Transmit the read instruction, address, and receive register value
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);						// Wait until SPI communication is finished
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave

	return rxBuf[2];	// Return register value
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
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, sizeof(txBuf)); 			// Transmit the write instruction, address, and command
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);						// Wait until SPI communication is finished
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave
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
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, sizeof(txBuf));			// Transmit the read instruction, address, and receive XYZ 8bit values
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);						// Wait until SPI communication is finished
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave

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
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, sizeof(txBuf));			// Transmit the read instruction, address, and receive XYZ 12bit values
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);						// Wait until SPI communication is finished
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave

	*x = ((int16_t)rxBuf[3] << 8) | (int16_t)rxBuf[2];
	*y = ((int16_t)rxBuf[5] << 8) | (int16_t)rxBuf[4];
	*z = ((int16_t)rxBuf[7] << 8) | (int16_t)rxBuf[6];
}

void ADXL362_GetXYZT(int16_t *x, int16_t *y, int16_t *z, int16_t *temp)
{
	uint8_t rxBuf[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t txBuf[10] = {0,0,0,0,0,0,0,0,0,0};

	txBuf[1] = XDATA_L;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, sizeof(txBuf));			// Transmit the read instruction, address, and receive XYZT 12bit values
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);						// Wait until SPI communication is finished
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave

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
	char msg[50];		// Message to print

	// Configure ADXL362_INT1
	ADXL362_AFlag = 0;		// Reset AWAKE flag

	// Disable ADXL362 External Interrupts
	HAL_NVIC_DisableIRQ(ADXL362_INT1_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(ADXL362_INT2_EXTI_IRQn);

	// Reset ADXL362 by writing 0x52(R in ASCII) to the Soft Reset Register
	sprintf(msg, "\r\nInitiating ADXL362!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	ADXL362_WriteReg(SOFT_RESET, 0x52);
	HAL_Delay(1);

	// Read ADXL362 registers
	reg = ADXL362_ReadReg(DEVID_AD); 	// Read ID Register
	sprintf(msg, "\r\nID0 = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	// Configure Activity and Inactivity Thresholds & Timers
	ADXL362_ActivityInit(USER_ACT_THRESH, USER_ACT_TIME);		// 300 code Activity Threshold. ODR = 100Hz, so 10 results & time threshold = 1 seconds
	ADXL362_InactivityInit(USER_INACT_THRESH, USER_INACT_TIME);	// 80 code Inactivity Threshold. ODR = 100Hz, so 30 results & time threshold = 2 seconds

	// Configure ADXL362 Activity/Inactivity Control Register
	ADXL362_WriteReg(ACT_INACT_CTL,0x3F);	// Set Referenced Activity and Inactivity, and Loop Mode
	reg = ADXL362_ReadReg(ACT_INACT_CTL);
	sprintf(msg, "\r\nACT_INACT_CTL = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	// Configure ADXL362 Interrupt Registers
	ADXL362_WriteReg(INTMAP1,0x40);		// Map Awake Interrupt to INT1 pin
	reg = ADXL362_ReadReg(INTMAP1);
	sprintf(msg, "\r\nINTMAP1 = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	ADXL362_WriteReg(INTMAP2,0x01);		// Map Data Ready Interrupt to INT2 pin
	reg = ADXL362_ReadReg(INTMAP2);
	sprintf(msg, "\r\nINTMAP2 = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	// Configure ADXL362 Filter Control Register
	ADXL362_WriteReg(FILTER_CTL,0x13);	// Set ADXL362 to 2g range, 25Hz
	reg = ADXL362_ReadReg(FILTER_CTL);
	sprintf(msg, "\r\nFILTER_CTL = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	// Configure ADXL362 Power Control Register
	ADXL362_WriteReg(POWER_CTL, 0x24);	// Set Autosleep bit POWER_CTL[2] and ultalow noise
	reg = ADXL362_ReadReg(POWER_CTL);
	sprintf(msg, "\r\nPOWER_CTL = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	ADXL362_WriteReg(POWER_CTL, reg | 0x02);	// Set to measurement mode
	reg = ADXL362_ReadReg(POWER_CTL);
	sprintf(msg, "\r\nPOWER_CTL = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	HAL_Delay(100);

	// Begin continuous processing of ADXL362 data
	HAL_NVIC_EnableIRQ(ADXL362_INT1_EXTI_IRQn);
	HAL_NVIC_EnableIRQ(ADXL362_INT2_EXTI_IRQn);

}

/**
  * @brief ADXL362 Activity Setup Function
  * @param thresh THRESH_ACT value for activity detection "sensitivity"
  * @param timer TIME_ACT value for activity detection "delay"
  * @retval None
  */
void ADXL362_ActivityInit (uint16_t thresh, uint8_t timer)
{
	uint8_t txBuf[5], rxBuf[5];

	txBuf[4] = timer;					// TIME_ACT
	txBuf[3] = (thresh >> 8) & 0xFF;	// THRESH_ACT_H
	txBuf[2] = thresh & 0xFF;			// THRESH_ACT_L
	txBuf[1] = THRESH_ACT_L;			// Address
	txBuf[0] = WR_ADXL;					// Write instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, sizeof(txBuf)); 			// Transmit the write instruction, address, THRESH_ACT, and TIME_ACT
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);						// Wait until SPI communication is finished
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave
}

/**
  * @brief ADXL362 Inactivity Setup Function
  * @param thresh THRESH_ACT value for inactivity detection "sensitivity"
  * @param timer TIME_ACT value for inactivity detection "delay"
  * @retval None
  */
void ADXL362_InactivityInit (uint16_t thresh, uint16_t timer)
{
	uint8_t txBuf[6], rxBuf[6];

	txBuf[5] = (timer >> 8) & 0xFF;		// TIME_INACT_H
	txBuf[4] = timer & 0xFF;			// TIME_INACT_L
	txBuf[3] = (thresh >> 8) & 0xFF;	// THRESH_INACT_H
	txBuf[2] = thresh & 0xFF;			// THRESH_INACT_L
	txBuf[1] = THRESH_INACT_L;			// Address
	txBuf[0] = WR_ADXL;					// Write instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, sizeof(txBuf)); 			// Transmit the write instruction, address, THRESH_INACT, and TIME_INACT
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);						// Wait until SPI communication is finished
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave
}
