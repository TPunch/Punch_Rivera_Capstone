#ifndef INCLUDE_ADXL362_H_
#define INCLUDE_ADXL362_H_

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "main.h"

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
// ADXL362 SPI Commands
#define WR_ADXL 		0x0A
#define RD_ADXL 		0x0B
#define FIFO_ADXL		0x0D

/* Function prototypes -----------------------------------------------*/
uint8_t ADXL362_ReadReg(uint8_t address);
void ADXL362_WriteReg(uint8_t address, uint8_t cmd);
void ADXL362_GetXYZ8(int8_t *x, int8_t *y, int8_t *z);
void ADXL362_GetXYZ12(int16_t *x, int16_t *y, int16_t *z);
void ADXL362_GetXYZT12(int16_t *x, int16_t *y, int16_t *z, int16_t *temp);
void ADXL362_Init(void);

#endif /* INCLUDE_ADXL362_H_ */
