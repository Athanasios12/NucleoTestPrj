#ifndef STM32F0XX_HAL_H
#define STM32F0XX_HAL_H

#include "stm32Hal_SPI_Stubs.h"
#include "stm32Hal_GPIO_Stubs.h"
#include "stm32Hal_NVIC_Stubs.h"
#include "stm32Hal_RCC_Stubs.h"

extern volatile bool spi_tx_done;
extern volatile bool spi_rx_done;



#endif //STM32F0XX_HAL_H