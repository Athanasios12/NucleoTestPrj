/*
 * dht11.c
 *
 *  Created on: Nov 18, 2020
 *      Author: radoslaw.rajczyk
 */
#include "stm32f0xx_hal.h"
#include "dht11.h"

static TIM_HandleTypeDef dht11Timer = {0};
static GPIO_TypeDef *DHT11_GPIO_PORT = NULL; // C
static uint16_t DHT11_GPIO_PIN = 0xFF; //PC0
const uint32_t initDelay = 18; //ms
static bool DHT11__Initalized = false;
#define DHT11_DATA_SIZE 5U

static void DHT11__delayMicroSec(uint16_t time);
static void DHT11__SetDataPin(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_Pin);
static bool DHT11__initTimer(TIM_TypeDef *timerID);
static bool DHT11__CheckResponse(void);
static void ReadDHT11Data(DHT11_Data *sensorData, GPIO_TypeDef *port, uint16_t pin);


static void DHT11__delayMicroSec(uint16_t time)
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
	if(GPIO_PORT == GPIOA)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}
	else if(GPIO_PORT == GPIOB)
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	else if(GPIO_PORT == GPIOC)
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
}

static bool DHT11__initTimer(TIM_TypeDef *timerID)
{
	bool timerInitSuccess = false;

	dht11Timer.Instance = timerID;
	dht11Timer.Init.Prescaler = 48 - 1; //base is 48Mhz so 1Mhz - 1 tick is 1 microsecond
	dht11Timer.Init.CounterMode = TIM_COUNTERMODE_UP;
	dht11Timer.Init.Period = 0xFFFF - 1;
	dht11Timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	dht11Timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_OK == HAL_TIM_Base_Init(&dht11Timer))
	{
		timerInitSuccess = true;
	}
	return timerInitSuccess;
}

static bool DHT11__CheckResponse(void)
{
	bool success = false;
	//check the response - wait 40us
	DHT11__delayMicroSec(40);
	//read the pin - it should be low at this point
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN))
	{
		//wait for 80us
		DHT11__delayMicroSec(80);
		//after this time the pin should be high
		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN))
		{
			//sensor is present return true
			success = true;
			while (GPIO_PIN_SET == HAL_GPIO_ReadPin (DHT11_GPIO_PORT, DHT11_GPIO_PIN));
		}
	}
	return success;
}

inline void set_Pin_Output(GPIO_TypeDef *port, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}

inline void set_Pin_Input(GPIO_TypeDef *port, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(port, &GPIO_InitStruct);


}

static void ReadDHT11Data(DHT11_Data *sensorData, GPIO_TypeDef *port, uint16_t pin)
{
	if (NULL != sensorData)
	{

	}
}

bool DHT11_Init(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_Pin, TIM_TypeDef *timerID)
{
	if (false == DHT11__Initalized)
	{
		DHT11__SetDataPin(GPIO_PORT, GPIO_Pin);
		if ((GPIO_Pin != 0xFF) && (NULL != GPIO_PORT))
		{
			if (DHT11__initTimer(timerID))
			{
				DHT11__Initalized = true;
			}
		}
	}
	return DHT11__Initalized;
}

bool DHT11_ReadDHT11Data(DHT11_Data *sensorData)
{
	bool dataReadSuccess = false;
	if (true == DHT11__Initalized)
	{
		HAL_TIM_Base_Start(&dht11Timer);
		GPIO_InitTypeDef GPIO_InitStruct = {0};

		GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
		//set pin low and wait 18ms
		HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET);
		HAL_Delay(18);
		//release the pin  - set as input
		HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_SET);
		DHT11__delayMicroSec(20);
		GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);

		DHT11__Initalized = DHT11__CheckResponse();

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
				DHT11__delayMicroSec(40);
				if (GPIO_PIN_SET == HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN))
				{
					readData |= 1 << (7 - bitNum);
				}
				while (GPIO_PIN_SET == HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN));
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
		HAL_TIM_Base_Stop(&dht11Timer);
	}
	return dataReadSuccess;
}
