/*
 * SPI_Handler.c
 *
 *  Created on: Feb 17, 2021
 *      Author: radoslaw.rajczyk
 */
#include "SPI_Handler.h"
#include "stm32f0xx_hal.h"
#include <string.h>

SPI_HandleTypeDef hspi2;
static SPI_State SPI_SM_State = SPI_OFF;
static uint8_t spi_buff[SPI_BUFF_SIZE] = {0};

volatile bool spi_tx_done = false;
volatile bool spi_rx_done = false;

bool SPI_TransmitData = false;
bool SPI_ReceiveData = false;


// This is called when SPI transmit is done
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (SPI2 == hspi->Instance)
	{
		// Set CS pin to high and raise flag
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		spi_tx_done = true;
	}
}

// This is called when SPI receive is done
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (SPI2 == hspi->Instance)
	{
		// Set CS pin to high and raise flag
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		spi_rx_done = true;
	}
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

		//CS - use software Chip select not hardware interrupt
		//later change to hardware IT
		gpio.Mode = GPIO_MODE_OUTPUT_PP;
		gpio.Pin = GPIO_PIN_8; // CS
		HAL_GPIO_Init(GPIOB, &gpio);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

		//SPI configuration
		hspi2.Instance = SPI2;
		hspi2.Init.Mode = SPI_MODE_MASTER;
		hspi2.Init.Direction = SPI_DIRECTION_2LINES;
		hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
		hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi2.Init.NSS = SPI_NSS_SOFT;
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

bool SPI_Init()
{
	bool spi_comm_initalized = false;
	if (SPI_INIT == SPI_SM_State)
	{
		//For now set the STM MCU as SPI master in comm with Rpi
		//Initial communication with the RPI - blocking, check if Rpi Present
		uint8_t spi_init_buff[] = "SPI COMM INIT";
		uint8_t spi_comm_established[] = "SPI RPI PRESENT";

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, spi_init_buff, strlen(spi_init_buff), HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, spi_buff, strlen(spi_comm_established), HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

		if (0 == strncmp(spi_buff, spi_comm_established, strlen(spi_comm_established)))
		{
			SPI_SM_State = SPI_IDLE;
			spi_comm_initalized = true;
		}
	}
	return spi_comm_initalized;
}

void SPI_CommSM()
{
	switch(SPI_SM_State)
	{
		case SPI_IDLE:
			if (SPI_TransmitData)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				SPI_TransmitData = false;
				SPI_SM_State = SPI_TX;
				HAL_SPI_Transmit_IT(&hspi2, spi_buff, SPI_BUFF_SIZE);
			}
			else if (SPI_ReceiveData)
			{
				//when the tx is done
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				SPI_SM_State = SPI_RX;
				SPI_ReceiveData = false;
				HAL_SPI_Receive_IT(&hspi2, spi_buff, SPI_BUFF_SIZE);
			}
			break;
		case SPI_RX:
			if (spi_rx_done)
			{
				SPI_SM_State = SPI_IDLE;
				spi_rx_done = false;
			}
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
