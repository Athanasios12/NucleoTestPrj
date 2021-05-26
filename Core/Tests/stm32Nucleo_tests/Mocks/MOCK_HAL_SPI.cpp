#include "Mock_HAL_SPI.h"

static Mock_HAL_SPI *MockInstance = nullptr;

Mock_HAL_SPI::Mock_HAL_SPI()
{
    MockInstance = this;
}

Mock_HAL_SPI::~Mock_HAL_SPI()
{
    MockInstance = nullptr;
}

HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    return MockInstance->HAL_SPI_Transmit_IT(hspi, pData, Size);
}

HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    return MockInstance->HAL_SPI_Receive_IT(hspi, pData, Size);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
    uint16_t Size)
{
    return MockInstance->HAL_SPI_TransmitReceive_IT(hspi, pTxData, pRxData, Size);
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef* hspi)
{
    return MockInstance->HAL_SPI_Init(hspi);
}