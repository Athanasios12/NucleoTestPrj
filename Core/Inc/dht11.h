/*
 * dht11.h - temperature and humidity sensor lib
 *
 *  Created on: Nov 18, 2020
 *      Author: radoslaw.rajczyk
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_
#include <stdbool.h>

typedef struct
{
	uint8_t temp_int;
	uint8_t temp_dec;
	uint8_t rh_int;
	uint8_t rh_dec;
	uint8_t checksum;
} DHT11_Data;

bool DHT11_Init(GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_Pin, TIM_TypeDef *timerID);
bool DHT11_ReadDHT11Data(DHT11_Data *sensorData);

#endif /* INC_DHT11_H_ */
