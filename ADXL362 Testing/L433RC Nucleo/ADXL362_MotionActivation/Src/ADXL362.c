/* Includes ------------------------------------------------------------------*/
#include "ADXL362.h"

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;

/**
  * @brief ADXL362 Read Register Function
  * @param address Address of register to read
  * @param cmd Command byte to read to register
  * @retval None
  */
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
  * @param x Points to data buffer for X-axis
  * @param y Points to data buffer for Y-axis
  * @param z Points to data buffer for Z-axis
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

	// Store sensor data in buffers
	*x = rxBuf[2];
	*y = rxBuf[3];
	*z = rxBuf[4];
}

/**
  * @brief ADXL362 Get XYZ 12bits Function
  * @param x Points to data buffer for X-axis
  * @param y Points to data buffer for Y-axis
  * @param z Points to data buffer for Z-axis
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

	// Store sensor data in buffers
	*x = ((int16_t)rxBuf[3] << 8) | (int16_t)rxBuf[2];
	*y = ((int16_t)rxBuf[5] << 8) | (int16_t)rxBuf[4];
	*z = ((int16_t)rxBuf[7] << 8) | (int16_t)rxBuf[6];
}

/**
  * @brief ADXL362 Get XYZT 12bits Function
  * @param xyzt Points to data buffer for XYZ-axes' forces and temperature
  * @param offset Offset for xyzt buffer
  * @retval None
  */
void ADXL362_GetXYZT(int16_t *xyzt, uint16_t offset)
{
	uint8_t rxBuf[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t txBuf[10] = {0,0,0,0,0,0,0,0,0,0};

	txBuf[1] = XDATA_L;		// Address
	txBuf[0] = RD_ADXL;		// Read instruction

	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_RESET);	// Pull CS pin low to enable the slave
	HAL_SPI_TransmitReceive_DMA(&hspi2, txBuf, rxBuf, sizeof(txBuf));			// Transmit the read instruction, address, and receive XYZT 12bit values
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);						// Wait until SPI communication is finished
	HAL_GPIO_WritePin(ADXL362_CS_GPIO_Port, ADXL362_CS_Pin, GPIO_PIN_SET);		// Pull CS pin high to disable the slave

	// Store sensor data in buffers
	xyzt[offset] = ((int16_t)rxBuf[3] << 8) | (int16_t)rxBuf[2];
	xyzt[offset + 1] = ((int16_t)rxBuf[5] << 8) | (int16_t)rxBuf[4];
	xyzt[offset + 2] = ((int16_t)rxBuf[7] << 8) | (int16_t)rxBuf[6];
	xyzt[offset + 3] = ((int16_t)rxBuf[9] << 8) | (int16_t)rxBuf[8];
}

/**
  * @brief ADXL362 Get AngT Function
  * @param xyzt Points to data buffer for XYZ angles and temperature
  * @param offset Offset for xyzt buffer
  * @retval None
  */
void ADXL362_GetAngT(float *xyzt, uint16_t offset)
{
	int16_t itemp[4];
	float ftemp[4];

	// Get X,Y,Z acceleration as ADC values
	ADXL362_GetXYZT(itemp, 0);

	// Process XYZT values to g's (acceleration due to gravity) 1mg = 1LSB
	ftemp[0] = (G_LSB * (float)itemp[0]);				// Offsets are due to using a supply voltage of ~3.3VDC
	ftemp[1] = (G_LSB * (float)itemp[1]);				// Offsets were adjusted primarily from the datasheet
	ftemp[2] = (G_LSB * (float)itemp[2]);				// then manually after
	ftemp[3] = (T_LSB * (float)itemp[3]) + TEMP_OFFSET;			// 0.065 degrees C = 1LSB

	// Process g's to angles x-axis angle(Rho), y-axis angle(Phi), and z-axis angle(Theta)
	xyzt[offset] = atan2f(ftemp[0], sqrt(pow(ftemp[1],2) + pow(ftemp[2], 2)));
	xyzt[offset] *= G_SCALER*180/M_PI;
	xyzt[1 + offset] = atan2f(ftemp[1], sqrt(pow(ftemp[0],2) + pow(ftemp[2], 2)));
	xyzt[1 + offset] *= G_SCALER*180/M_PI;
	xyzt[2 + offset] = atan2f(sqrt(pow(ftemp[0], 2) + pow(ftemp[1], 2)), ftemp[2]);
	xyzt[2 + offset] *= G_SCALER*180/M_PI;

	// Convert degrees C to degrees F
	xyzt[3 + offset] = (ftemp[3] * (9.0/5.0)) + 32;
}

/**
  * @brief ADXL362 Get Tilt State Function
  * @param ang Points to data buffer with XYZ angles and temperature
  * @param offset Offset for ang buffer
  * @param thresh Points to data buffer with XYZ threshold
  * @param tilt Points to data buffer for garage tilt angles
  * @param GarageState Points to data buffer for Garage State
  * @retval None
  */
void ADXL362_GetTiltState(float *ang, uint16_t offset, float *thresh, float *tilt, uint8_t *GarageState)
{
	// Calculate tilt angles by using threshold angles as reference from
	tilt[0] = ((thresh[0]) - (ang[offset]) + X_OFFSET);
	tilt[1] = ((thresh[1]) - (ang[1+offset]) + Y_OFFSET);
	tilt[2] = ((thresh[2]) - (ang[2+offset]) + Z_OFFSET);

	// Determine garage door state by comparing threshold and current angles
	if((tilt[0] > T_ANG) || (tilt[1] > T_ANG) || (tilt[2] > T_ANG)){
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	  *GarageState = 1;
	} else{
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	  *GarageState = 0;
	}
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

	// Disable ADXL362 External Interrupts
	HAL_NVIC_DisableIRQ(ADXL362_INT1_EXTI_IRQn);

	// Reset ADXL362 by writing 0x52(R in ASCII) to the Soft Reset Register
	sprintf(msg, "\r\nInitiating ADXL362!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	ADXL362_WriteReg(SOFT_RESET, 0x52);
	HAL_Delay(1);

	// Read ADXL362 registers

	// Read ID Register
	reg = ADXL362_ReadReg(DEVID_AD);
	sprintf(msg, "\r\nID0 = 0x%X\r\n", reg);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);

	// Configure Activity and Inactivity Thresholds & Timers
	ADXL362_ActivityInit(USER_ACT_THRESH, USER_ACT_TIME);		// 50 code Activity Threshold.
	ADXL362_InactivityInit(USER_INACT_THRESH, USER_INACT_TIME);	// 50 code Inactivity Threshold.

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

	// Configure ADXL362 Filter Control Register
	ADXL362_WriteReg(FILTER_CTL,0x13);	// Set ADXL362 to 2g range, 100Hz
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
