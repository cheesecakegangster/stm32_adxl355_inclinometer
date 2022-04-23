/*
 *  ADXL355_SPI.c
 *  Analog Devices ADXL355 SPI HAL Driver using DMA
 *
 *  Created on: Apr 22, 2022
 *  Author: Dusan Pesic
 */

#include "ADXL355_SPI.h"

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
	uint8_t rxdata;
	status = ADXL355_ReadRegister(device, ADXL355_DEVID_ID, &rxdata);
	errNum += (status != HAL_OK);

	if (rxdata != ADXL355_DEVID_ID_VAL){
		return rxdata;
	}
	else
	{
		return rxdata;
	}
	//return 0;
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
	return HAL_SPI_TransmitReceive(device->hspi, &txdata, rxdata, 1, SPI_TIMEOUT); //(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
}
//HAL_StatusTypeDef ADXL355_ReadRegisters(ADXL355_type *device, uint8_t reg, uint8_t *data, uint8_t len)
//{
//	//return HAL_SPI_Transmit();
//}
//HAL_StatusTypeDef ADXL355_WriteRegister(ADXL355_type *device, uint8_t reg, uint8_t *data)
//{
//	//return HAL_SPI_Transmit();
//}
