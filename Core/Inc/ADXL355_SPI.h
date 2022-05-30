/*
 *  ADXL355_SPI.h
 *  Analog Devices ADXL355 SPI HAL Driver
 *
 *  Created on: Apr 22, 2022
 *  Author: Dusan Pesic
 */

#include "stm32f4xx_hal.h"

#ifndef ADXL355_SPI_H
#define ADXL355_SPI_H

// ADXL355 static definitions
#define ADXL355_DEVID_ID_VAL	0xAD // Device ID
#define ADXL355_DEVID_MST_VAL 	0x1D // MEMS ID
#define ADXL355_PARTID_VAL		0xED // Part ID
#define ADXL355_REVID_VAL		0x01 // Product revision ID

// ADXL355 register map
#define ADXL355_DEVID_ID		0x00 // Device ID
#define ADXL355_DEVID_MST 		0x01 // MEMS ID
#define ADXL355_PARTID			0x02 // Part ID
#define ADXL355_REVID			0x03 // Product revision ID
#define ADXL355_STATUS			0x04 // Status register: 0 = DATA_RDY, 1=FIFO_FULL, 2=FIFO_OVR, 3=Activity detected, 4=NVM_BUSY
#define ADXL355_FIFO_ENTRIES	0x05 // This register indicates the number of valid data samples present in the FIFO buffer. This number ranges from 0 to 96.
#define ADXL355_TEMP2			0x06 // These two registers contain the uncalibrated temperature data (12 bits)
#define ADXL355_TEMP1			0x07 // TEMP2 contains the four most significant bits, and TEMP1 contains the eight least significant bits
#define ADXL355_XDATA3			0x08 // XDATA, Bits[19:12] Acceleration data is left justified and formatted as twos complement.
#define ADXL355_XDATA2			0x09 // XDATA, Bits[11:4]
#define ADXL355_XDATA1			0x0A // XDATA, Bits[3:0]
#define ADXL355_YDATA3			0x0B // YDATA, Bits[19:12]
#define ADXL355_YDATA2			0x0C // YDATA, Bits[11:4]
#define ADXL355_YDATA1			0x0D // YDATA, Bits[3:0]
#define ADXL355_ZDATA3			0x0E // ZDATA, Bits[19:12]
#define ADXL355_ZDATA2			0x0F // ZDATA, Bits[11:4]
#define ADXL355_ZDATA1			0x10 // ZDATA, Bits[3:0]
#define ADXL355_FIFO_DATA		0x11 // Read this register to access data stored in the FIFO.
#define ADXL355_OFFSET_X_H		0x1E // Offset added to x-axis data after all other signal processing. Data is in twos complement format. The significance of OFFSET_X, Bits[15:0] matches the significance of XDATA, Bits[19:4].
#define ADXL355_OFFSET_X_L		0x1F //
#define ADXL355_OFFSET_Y_H		0x20 //
#define ADXL355_OFFSET_Y_L		0x21 //
#define ADXL355_OFFSET_Z_H		0x22 //
#define ADXL355_OFFSET_Z_L		0x23 //
#define ADXL355_ACT_EN			0x24 // 0: X-axis 1, Y-axis, 2: Z-axis -- is a component of the activity detection algorithm.
#define ADXL355_ACT_THRESH_H	0x25 // Threshold for activity detection. The acceleration magnitude must be greater than the value in ACT_THRESH to trigger the activity counter. ACT_THRESH is an unsigned magnitude
#define ADXL355_ACT_THRESH_L	0x26 //
#define ADXL355_ACT_COUNT		0x27 // Number of consecutive events above threshold (from ACT_THRESH) required to detect activity
#define ADXL355_FILTER			0x28 // Filter properties (see page 38 of 42 in ADXL355 datasheet)
#define ADXL355_FIFO_SAMPLES	0x29 // Use the FIFO_SAMPLES value to specify the number of samples to store in the FIFO. The default value of this register is 0x60 to avoid triggering the FIFO watermark interrupt.
#define ADXL355_INT_MAP			0x2A // The INT_MAP register configures the interrupt pins.
#define ADXL355_SYNC			0x2B // Use this register to control the external timing triggers.
#define ADXL355_RANGE			0x2C // I2C SPEED, INTERRUPT POLARITY, AND RANGE REGISTER, bits [1:0] are for range, 6 for interrupt polarity, 7 for i2c speed
#define ADXL355_POWER_CTL		0x2D // Bit 0: 0 = measurement mode, 1 = standby mode
#define ADXL355_SELF_TEST		0x2E //	Self test
#define ADXL355_RESET			0x2F // Reset device (write 0x52 to register)
#define SHADOW_REGISTERS_START	0x50 // See page 40 of 42 in ADXL355 datasheet. Only relevant is software resets are to be performed.
#define SHADOW_REGISTERS_END	0x54 //

typedef struct{
	SPI_HandleTypeDef *hspi;
	float acceleration_x_g;
	float acceleration_y_g;
	float acceleration_z_g;
	float temperature_deg_c;
}ADXL355_type;

// initialisation
uint8_t ADXL355_init(ADXL355_type *device, SPI_HandleTypeDef *hspi);

// data acquisition
HAL_StatusTypeDef ADXL355_ReadTemperature(ADXL355_type *device);		// reads the temperature registers
HAL_StatusTypeDef ADXL355_ReadAccelerations(ADXL355_type *device);		// reads all accelerometer registers of the ADXL

// low level functions
HAL_StatusTypeDef ADXL355_ReadRegister(ADXL355_type *device, uint8_t txdata, uint8_t *rxdata); // read specific register
HAL_StatusTypeDef ADXL355_MultiByteRead(ADXL355_type *device, uint8_t txdata, uint8_t *rxdata, uint8_t length); // read specific registers and walk through memory starting from the defined register
HAL_StatusTypeDef ADXL355_WriteRegister(ADXL355_type *device, uint8_t reg, uint8_t *data); // write to specific register


#endif /* ADXL355_SPI_H */
