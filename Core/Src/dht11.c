/*
 * dht11.c
 *
 *  Created on: Nov 18, 2020
 *      Author: radoslaw.rajczyk
 */
#include "stm32f0xx_hal.h"
#include "dht11.h"

TIM_HandleTypeDef dht11Timer;
static GPIO_TypeDef *DHT11_GPIO_PORT = NULL; // C
static uint16_t DHT11_GPIO_PIN = 0xFF; //PC0
const uint32_t initDelay = 18; //ms
#define DHT11_DATA_SIZE 5U

static void DHT__delayMicroSec(uint16_t time);
static void DHT11__SetDataPin(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_Pin);
static void DHT11__initTimer(TIM_TypeDef *timerID);
static bool DHT11__CheckResponse(void);


static void DHT__delayMicroSec(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&dht11Timer, 0);
	while (__HAL_TIM_GET_COUNTER(&dht11Timer) < time)
	{

	}
}

static void DHT11__SetDataPin(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_Pin)
{
	DHT11_GPIO_PORT = GPIO_PORT;
	DHT11_GPIO_PIN = GPIO_Pin;
}

static void DHT11__initTimer(TIM_TypeDef *timerID)
{
	TIM_HandleTypeDef dht11Timer = {0};
	dht11Timer.Instance = timerID;
	dht11Timer.Init.Prescaler = 48 - 1; //base is 48Mhz so 1Mhz - 1 tick is 1 microsecond
	dht11Timer.Init.CounterMode = TIM_COUNTERMODE_UP;
	dht11Timer.Init.Period = 0xFFFF - 1;
	dht11Timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	dht11Timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_MspPostInit(&dht11Timer);
}

static bool DHT11__CheckResponse(void)
{
	bool success = false;
	//check the response - wait 40us
	delayMicroSec(40);
	//read the pin - it should be low at this point
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN))
	{
		//wait for 80us
		delayMicroSec(80);
		//after this time the pin should be high
		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN))
		{
			//sensor is present return true
			success = true;
		}
	}
	return success;
}

bool DHT11_Init(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_Pin, TIM_TypeDef *timerID)
{
	bool success = false;
	if ((GPIO_Pin != 0xFF) && (NULL != GPIO_PORT))
	{
		DHT11__SetDataPin(GPIO_PORT, GPIO_Pin);
		DHT11__initTimer(timerID);

		GPIO_InitTypeDef GPIO_InitStruct = {0};

		GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
	    //set pin low and wait 18ms
	    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET);
	    HAL_Delay(18);
	    //release the pin  - set as input
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
	    success = DHT11_CheckResponse();
	}
	return success;
}

bool DHT11_ReadDHT11Data(DHT11_Data *sensorData)
{
	DHT11_Start();
	uint8_t data[DHT11_DATA_SIZE]; // [PRESINT, PRESDEC, TEMPINT, TEMPDEC, CHECKSUM]
	uint8_t readData = 0;
	for (uint8_t byteNum = 0; byteNum < DHT11_DATA_SIZE; byteNum++)
	{
		for (uint8_t bitNum = 0; bitNum < 8; bitNum++)
		{
			while (GPIO_PIN_SET != HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN))
			{
				//Transmission begins with 50us of low voltage level,
				//When the high occurs start read
			}
			//Next starts sending hight volage - if the high is 26-28us the bit is "0", if 70us then "1"
			//Wait 40us - if low it means that the bit is "0"
			delay(40);
			if (GPIO_PIN_SET == HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN))
			{
				readData |= 1 << (7 - bitNum);
			}
		}
		data[byteNum] = readData;
		readData = 0;
	}
	//check checksum byte: CHECKSUM = TEMP + PRES
	sensorData->rh_int = data[0];
	sensorData->rh_dec = data[1];
	sensorData->temp_int = data[2];
	sensorData->temp_dec = data[3];
	sensorData->checksum = data[4];
	return true;
}
