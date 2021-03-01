/*
 * SPI_Handler.h
 *
 *  Created on: Feb 17, 2021
 *      Author: radoslaw.rajczyk
 */

#ifndef INC_SPI_HANDLER_H_
#define INC_SPI_HANDLER_H_
#include <stdbool.h>

typedef enum
{
	SPI_OFF,
	SPI_INIT,
	SPI_IDLE,
	SPI_RX,
	SPI_TX,
} SPI_State;

#define SPI_BUFF_SIZE 50

void SPI_CommSM();
bool SPI_Config();
void SPI_Init();


#endif /* INC_SPI_HANDLER_H_ */
