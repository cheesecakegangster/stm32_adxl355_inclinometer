/*
 *  ADXL355_SPI.c
 *  Analog Devices ADXL355 SPI HAL Driver using DMA
 *
 *  Created on: Apr 22, 2022
 *  Author: Dusan Pesic
 */

#include "ADXL355_SPI.h"
#include "stdio.h"
#include "math.h"

#define SPI_TIMEOUT 1000

uint8_t ADXL355_init(ADXL355_type *device, SPI_HandleTypeDef *hspi, GPIO_TypeDef * nss_gpio_port, uint16_t nss_gpio_pin)
{
	device->hspi				= hspi;
	device->nss_gpio_port		= nss_gpio_port;
	device->nss_gpio_pin		= nss_gpio_pin;

	device->acceleration_x_g 	= 0.0f;
	device->acceleration_y_g 	= 0.0f;
	device->acceleration_z_g 	= 0.0f;

	device->temperature_deg_c	= 0.0f;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	// check if the correct device is connected
	uint8_t rxdata[3];														// array to store received data in
	status = ADXL355_MultiByteRead(device, ADXL355_DEVID_ID, rxdata, 3);	// perform multibyte read on first 3 registers which contain DEVICE ID, MEMS ID, PART ID
	errNum += (status != HAL_OK);											// add to error

	if (rxdata[0] != ADXL355_DEVID_ID_VAL)
	{
		return 255;
	}

	if (rxdata[1] != ADXL355_DEVID_MST_VAL)
	{
		return 254;
	}
	if (rxdata[2] != ADXL355_PARTID_VAL)
	{
		return 253;
	}

	//printf("%i\n\r", rxdata[0]);
	//HAL_Delay(1);
	//printf("%i\n\r", rxdata[1]);
	//HAL_Delay(1);
	//printf("%i\n\r", rxdata[2]);
	//HAL_Delay(1);

	status = ADXL355_SingleByteWrite(device, ADXL355_POWER_CTL, 0x00); // enable temperature measurement
	if (status != HAL_OK){
		return 252;
	}
	uint8_t reg_conf;
	status = ADXL355_SingleByteRead(device, ADXL355_POWER_CTL, &reg_conf);
	//printf("%i\n\r", reg_conf);
	//HAL_Delay(1);

	return HAL_OK;
}

HAL_StatusTypeDef ADXL355_ReadTemperature(ADXL355_type *device)
{
	//make array to store 2 temperature bytes in -> byte 0 is msb, byte 1 is lsb -> concat in 16 bit var -> calculate temp as float -> write to device.temperature
	//−9.05 LSB/°C
	//return HAL_SPI_Transmit();
	uint8_t rxdata2[2];

	HAL_StatusTypeDef status = ADXL355_MultiByteRead(device, ADXL355_TEMP2, rxdata2, 2);
	//printf("Transmission status read temp: %i\n\r", status);
	//HAL_Delay(1);
	if (status != HAL_OK){ // only perform calculation if the transmission was a success
		return status;
	}
	//printf("rxdata2[0] contents: %i\n\r",rxdata2[0]);
	//HAL_Delay(1);
	uint16_t temp_data = (uint16_t)rxdata2[0] << 8 | rxdata2[1];
	//printf("Temp data: %i\n\r", temp_data);
	//HAL_Delay(1);
	device->temperature_deg_c = (float)(25+((temp_data-1852)*(1/-9.05)));

	return status;
}

HAL_StatusTypeDef ADXL355_ReadAccelerations(ADXL355_type *device)
{
	uint8_t xyz_data[9];

	HAL_StatusTypeDef status = ADXL355_MultiByteRead(device, ADXL355_XDATA3, xyz_data, 9);
	int32_t x_raw = ((int32_t)xyz_data[0] << 24 | (int32_t)xyz_data[1] << 16 | xyz_data[2] << 8) >> 12;
	//printf("x acceleration data raw: %li\n\r", x_raw);
	//HAL_Delay(1);
	int32_t y_raw = ((int32_t)xyz_data[3] << 24 | (int32_t)xyz_data[4] << 16 | xyz_data[5] << 8) >> 12;
	//printf("y acceleration data raw: %li\n\r", y_raw);
	//HAL_Delay(1);
	int32_t z_raw = ((int32_t)xyz_data[6] << 24 | (int32_t)xyz_data[7] << 16 | xyz_data[8] << 8) >> 12;
	//printf("z acceleration data raw: %li\n\r", z_raw);
	//HAL_Delay(1);

	device->acceleration_x_g = (4.096f/(pow(2, 20))) * x_raw;
	device->acceleration_y_g = (4.096f/(pow(2, 20))) * y_raw;
	device->acceleration_z_g = (4.096f/(pow(2, 20))) * z_raw;

	//printf("device x acceleration = %f \n\r", device->acceleration_x_g);
	//HAL_Delay(1);
	//printf("device y acceleration = %f \n\r", device->acceleration_y_g);
	//HAL_Delay(1);
	//printf("device z acceleration = %f \n\r", device->acceleration_z_g);
	//HAL_Delay(1);

	return status;
}

// low level functions

HAL_StatusTypeDef ADXL355_SingleByteRead(ADXL355_type *device, uint8_t txdata, uint8_t *rxdata)
{
	HAL_GPIO_WritePin(device->nss_gpio_port, device->nss_gpio_pin, RESET);
	uint8_t txdata2 = (txdata << 1) + 1;							// the LSB is a read/write bit (write = 0, read = 1), so must be bit shifted << 1 and add 1 to read
	HAL_SPI_Transmit(device->hspi, &txdata2, 1, SPI_TIMEOUT);		// HAL_SPI_TransmitReceive doesn't work for some reason (only clocks 8 cycles -> is it because I used length = 1?) -> return HAL_SPI_TransmitReceive(device->hspi, &txdata2, rxdata, 1, SPI_TIMEOUT);
	HAL_StatusTypeDef status = HAL_SPI_Receive(device->hspi, rxdata, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(device->nss_gpio_port, device->nss_gpio_pin, SET);
	return status;
}

HAL_StatusTypeDef ADXL355_MultiByteRead(ADXL355_type *device, uint8_t txdata, uint8_t *rxdata, uint8_t length)
{
	HAL_GPIO_WritePin(device->nss_gpio_port, device->nss_gpio_pin, RESET);
	uint8_t txdata2 = (txdata << 1) + 1;
	HAL_SPI_Transmit(device->hspi, &txdata2, 1, SPI_TIMEOUT);
	HAL_StatusTypeDef status = HAL_SPI_Receive(device->hspi, rxdata, length, SPI_TIMEOUT); // walks through registers in order from start point "txdata" with specified length in bytes
	HAL_GPIO_WritePin(device->nss_gpio_port, device->nss_gpio_pin, SET);
	return status;
}

HAL_StatusTypeDef ADXL355_SingleByteWrite(ADXL355_type *device, uint8_t reg, uint8_t txdata)
{
	HAL_GPIO_WritePin(device->nss_gpio_port, device->nss_gpio_pin, RESET);
	uint8_t reg2 = (reg << 1);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(device->hspi, &reg2, 1+1, SPI_TIMEOUT);
	status = HAL_SPI_Transmit(device->hspi, &txdata, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(device->nss_gpio_port, device->nss_gpio_pin, SET);
	return status;
}


HAL_StatusTypeDef ADXL355_MultiByteWrite(ADXL355_type *device, uint8_t reg, uint8_t txdata, uint8_t length)
{
	HAL_GPIO_WritePin(device->nss_gpio_port, device->nss_gpio_pin, RESET);
	uint8_t reg2 = (reg << 1);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(device->hspi, &reg2, length+1, SPI_TIMEOUT);
	status = HAL_SPI_Transmit(device->hspi, &txdata, length, SPI_TIMEOUT);
	HAL_GPIO_WritePin(device->nss_gpio_port, device->nss_gpio_pin, SET);
	return status;
}
