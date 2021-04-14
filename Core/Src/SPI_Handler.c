/*
 * SPI_Handler.c
 *
 *  Created on: Feb 17, 2021
 *      Author: radoslaw.rajczyk
 */
#include "SPI_Handler.h"
#include "stm32f0xx_hal.h"
#include <string.h>

typedef enum
{
	RPI_INIT,
	STM_CONNECTED_ACK,
	RPI_GET_DATE_TIME,
	SEND_SENSOR_DATA
} SPI_Command;

SPI_HandleTypeDef hspi2;
static SPI_State SPI_SM_State = SPI_OFF;
static uint8_t spi_tx_buff[SPI_TX_BUFF_SIZE] = {0};
static uint8_t spi_rx_buff[SPI_RX_BUFF_SIZE] = {0};
const char* spi_commands[] =
{
	"RPI_INI",
	"STM_ACK",
	"STM_GET",
	"RPI_GET"
};

volatile bool spi_tx_done = false;
volatile bool spi_rx_done = false;
bool spi_commEstablished = false;

static bool checkIfConnectedRpi(void);

// This is called when SPI transmit is done
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (SPI2 == hspi->Instance)
	{
		spi_tx_done = true;
	}
}

// This is called when SPI receive is done
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (SPI2 == hspi->Instance)
	{
		spi_rx_done = true;
	}
}

SPI_State SPI_getState()
{
	return SPI_SM_State;
}

bool SPI_Config()
{
	bool spi_config_success = false;
	if (SPI_OFF == SPI_SM_State)
	{
		//SPI2 CLK ENABLE
		__HAL_RCC_SPI2_CLK_ENABLE();

		//SCLK, MOSI
		__HAL_RCC_GPIOB_CLK_ENABLE();

		GPIO_InitTypeDef gpio;
		gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; //SCLK, MISO, MOSI
		gpio.Mode = GPIO_MODE_AF_PP;
		gpio.Pull = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_FREQ_HIGH;
		gpio.Alternate = GPIO_AF0_SPI2;
		HAL_GPIO_Init(GPIOB, &gpio);


		//SPI configuration
		hspi2.Instance = SPI2;
		hspi2.Init.Mode = SPI_MODE_SLAVE;
		hspi2.Init.Direction = SPI_DIRECTION_2LINES;
		hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
		hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
		hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;// 1.5MHz
		hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
		hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi2.Init.CRCPolynomial = 7;
		hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

		/* SPI2 interrupt Init */
		HAL_NVIC_SetPriority(SPI2_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(SPI2_IRQn);

		if (HAL_SPI_Init(&hspi2) == HAL_OK)
		{
			SPI_SM_State = SPI_INIT;
			spi_config_success = true;
		}
	}
	return spi_config_success;
}

void SPI_Init()
{
	if (!spi_commEstablished)
	{
		SPI_SM_State = SPI_RX;
		HAL_SPI_Receive_IT(&hspi2, spi_rx_buff, SPI_RX_BUFF_SIZE);
	}
}

bool SPI_PrepareSensorDataTransmit(SPI_TxSensorData *txData)
{
	if (SPI_SM_State == SPI_IDLE && spi_commEstablished)
	{
		memcpy((uint8_t*)txData, spi_tx_buff, sizeof(SPI_TxSensorData));
		SPI_SM_State = SPI_TX;
		HAL_SPI_Transmit_IT(&hspi2, spi_tx_buff, SPI_TX_BUFF_SIZE);
		return true;
	}
	return false;
}

bool SPI_PrepareReadTransmitData()
{
	if (SPI_SM_State == SPI_IDLE && spi_commEstablished)
	{
		SPI_SM_State = SPI_RX;
		if (HAL_OK == HAL_SPI_Receive_IT(&hspi2, spi_rx_buff, SPI_RX_BUFF_SIZE))
		{
			return true;
		}
	}
	return false;
}

bool SPI_ReadTransmitData(SPI_RxDateTime *rxData)
{
	if (SPI_SM_State == SPI_RX_DATA_AVAILABLE)
	{
		memcpy((uint8_t*)rxData, spi_rx_buff, sizeof(SPI_RxDateTime));
		//only after data was read the state changes to idle again and enables starting next read
		SPI_SM_State = SPI_IDLE;
		return true;
	}
	return false;
}

bool SPI_RequestDateTimeFromRpi()
{
	if (SPI_SM_State == SPI_IDLE && spi_commEstablished)
	{
		memcpy(spi_commands[RPI_GET_DATE_TIME], spi_tx_buff,
			strlen(spi_commands[RPI_GET_DATE_TIME]));
		SPI_SM_State = SPI_TX;
		HAL_SPI_Transmit_IT(&hspi2, spi_tx_buff, SPI_TX_BUFF_SIZE);
		return true;
	}
	return false;
}

void SPI_CommSM() // called cyclicly in main
{
	switch(SPI_SM_State)
	{
		case SPI_IDLE:
			break;
		case SPI_RX:
			if (spi_rx_done)
			{
				spi_rx_done = false;
				if (!spi_commEstablished)
				{
					SPI_SM_State = SPI_IDLE;
					//check rx data if connected to RPi
					spi_commEstablished = checkIfConnectedRpi();
					if (spi_commEstablished)
					{
						SPI_SM_State = SPI_TX;
						memcpy(spi_commands[STM_CONNECTED_ACK], spi_tx_buff,
							strlen(spi_commands[STM_CONNECTED_ACK]));
						HAL_SPI_Transmit_IT(&hspi2, spi_tx_buff, SPI_TX_BUFF_SIZE);
					}
				}
				else
				{
					SPI_SM_State = SPI_RX_DATA_AVAILABLE;
				}
			}
			break;
		case SPI_RX_DATA_AVAILABLE:
			break;
		case SPI_TX:
			if (spi_tx_done)
			{
				SPI_SM_State = SPI_IDLE;
				spi_tx_done = false;
			}
			break;
		default:
			break;
	}
}

static bool checkIfConnectedRpi(void)
{
	if (!spi_commEstablished)
	{
		if (strncmp(spi_tx_buff, spi_commands[RPI_INIT],
			strlen(spi_commands[RPI_INIT])))
		{
			return true;
		}
	}
	else
	{
		return true;
	}
	return false;
}
