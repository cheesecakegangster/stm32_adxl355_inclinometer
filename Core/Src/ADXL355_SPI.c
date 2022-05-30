/*
 *  ADXL355_SPI.c
 *  Analog Devices ADXL355 SPI HAL Driver using DMA
 *
 *  Created on: Apr 22, 2022
 *  Author: Dusan Pesic
 */

#include "ADXL355_SPI.h"
#include "stdio.h"

#define SPI_TIMEOUT 1000

uint8_t ADXL355_init(ADXL355_type *device, SPI_HandleTypeDef *hspi)
{
	device->hspi				= hspi;

	device->acceleration_x_g 	= 0.0f;
	device->acceleration_y_g 	= 0.0f;
	device->acceleration_z_g 	= 0.0f;

	device->temperature_deg_c	= 0.0f;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	// check if the correct device is connected
	//uint8_t rxdata = 0;
	uint8_t rxdata2[3];
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
	status = ADXL355_MultiByteRead(device, ADXL355_DEVID_ID, rxdata2, 3);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
	//status = ADXL355_ReadRegister(device, ADXL355_DEVID_ID, &rxdata);
	errNum += (status != HAL_OK);

//	for (int i = 0; i < sizeof(rxdata2)/sizeof(rxdata2[0]); i++)
//	{
//		printf("%d\n", rxdata2[i]);
//		printf("lol");
//	}

	if (rxdata2[0] != ADXL355_DEVID_ID_VAL)
	{
//		printf("rxdata2[0] = %i", rxdata2[0]);
		return 255;
	}

	if (rxdata2[1] != ADXL355_DEVID_MST_VAL)
	{
//		printf("rxdata2[1] = %i", rxdata2[1]);
		return 254;
	}
	if (rxdata2[2] != ADXL355_PARTID_VAL)
	{
//		printf("rxdata2[2] = %i", rxdata2[2]);
		return 253;
	}
	else
	{
		printf("%i\n", rxdata2[0]);
		HAL_Delay(1);
		printf("%i\n", rxdata2[1]);
		HAL_Delay(1);
		printf("%i\n", rxdata2[2]);
		HAL_Delay(1);
		return HAL_OK;
	}
}

//HAL_StatusTypeDef ADXL355_ReadTemperature(ADXL355_type *device)
//{
//	//return HAL_SPI_Transmit();
//}
//HAL_StatusTypeDef ADXL355_ReadAccelerations(ADXL355_type *device)
//{
//	//return HAL_SPI_Transmit();
//}

// low level functions

HAL_StatusTypeDef ADXL355_ReadRegister(ADXL355_type *device, uint8_t txdata, uint8_t *rxdata)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
	uint8_t txdata2 = (txdata << 1) + 1;							// the LSB is a read/write bit (write = 0, read = 1), so must be bit shifted << 1 and add 1 to read
	HAL_SPI_Transmit(device->hspi, &txdata2, 1, SPI_TIMEOUT);		// HAL_SPI_TransmitReceive doesn't work for some reason (only clocks 8 cycles -> is it because I used length = 1?) -> return HAL_SPI_TransmitReceive(device->hspi, &txdata2, rxdata, 1, SPI_TIMEOUT);
	HAL_StatusTypeDef status = HAL_SPI_Receive(device->hspi, rxdata, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
	return status;
}

HAL_StatusTypeDef ADXL355_MultiByteRead(ADXL355_type *device, uint8_t txdata, uint8_t *rxdata, uint8_t length)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
	uint8_t txdata2 = (txdata << 1) + 1;
	HAL_SPI_Transmit(device->hspi, &txdata2, 1, SPI_TIMEOUT);
	HAL_StatusTypeDef status = HAL_SPI_Receive(device->hspi, rxdata, length, SPI_TIMEOUT); // walks through registers in order from start point "txdata" with specified length in bytes
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
	return status;
}

//HAL_StatusTypeDef ADXL355_ReadRegisters(ADXL355_type *device, uint8_t reg, uint8_t *data, uint8_t len)
//{
//	//return HAL_SPI_Transmit();
//}
//HAL_StatusTypeDef ADXL355_WriteRegister(ADXL355_type *device, uint8_t reg, uint8_t *data)
//{
//	//return HAL_SPI_Transmit();
//}
