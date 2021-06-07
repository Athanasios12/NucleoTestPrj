/*
 * SPI_Handler.h
 *
 *  Created on: Feb 17, 2021
 *      Author: radoslaw.rajczyk
 */

#ifndef INC_SPI_HANDLER_H_
#define INC_SPI_HANDLER_H_
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	SPI_OFF,
	SPI_INIT,
	SPI_IDLE,
	SPI_RX,
	SPI_RX_DATA_AVAILABLE,
	SPI_TX,
} SPI_State;

typedef struct
{
	uint8_t humidity_int;
	uint8_t humidity_dec;
	uint8_t temp_int;
	uint8_t temp_dec;
	uint8_t adcVoltage : 3;
	uint8_t moveSensorState : 1;
} SPI_TxSensorData;

typedef struct
{
	//Date and time data
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} SPI_RxDateTime;

#define SPI_RX_BUFF_SIZE 10
#define SPI_TX_BUFF_SIZE 10

void SPI_CommSM();
bool SPI_Config();
void SPI_Init();
bool SPI_CommEstablished();
bool SPI_PrepareSensorDataTransmit(SPI_TxSensorData *txData);
bool SPI_ReadTransmitData(SPI_RxDateTime *rxData);
bool SPI_RequestDateTimeFromRpi();
SPI_State SPI_getState();


#endif /* INC_SPI_HANDLER_H_ */
