#ifndef MOCK_HAL_SPI_H
#define MOCK_HAL_SPI_H
#include <gmock/gmock.h>
extern "C"
{
#include "stm32Hal_SPI_Stubs.h"
}

class Mock_HAL_SPI
{
public:
    Mock_HAL_SPI();
    virtual ~Mock_HAL_SPI();

    MOCK_METHOD(HAL_StatusTypeDef, HAL_SPI_Init, (SPI_HandleTypeDef*));
    MOCK_METHOD(HAL_StatusTypeDef, HAL_SPI_Transmit_IT, (SPI_HandleTypeDef *, uint8_t *, uint16_t ));
    MOCK_METHOD(HAL_StatusTypeDef, HAL_SPI_Receive_IT, (SPI_HandleTypeDef *, uint8_t *, uint16_t));
    MOCK_METHOD(HAL_StatusTypeDef, HAL_SPI_TransmitReceive_IT, (SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t));
};

#endif //MOCK_HAL_SPI_H