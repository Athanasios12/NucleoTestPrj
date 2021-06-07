#include "Mock_HAL_SPI.h"

static Mock_HAL_SPI *MockInstance = nullptr;
SPI_HandleTypeDef hspi2;

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

void Mock_HAL_SPI::resetSpiConfig() const
{
    hspi2.Instance = NoDevice;
    hspi2.Init.Mode = 0;
    hspi2.Init.Direction = 0;
    hspi2.Init.DataSize = 0;
    hspi2.Init.CLKPolarity = 0;
    hspi2.Init.CLKPhase = 0;
    hspi2.Init.NSS = 0;
    hspi2.Init.BaudRatePrescaler = 0;
    hspi2.Init.FirstBit = 0;
    hspi2.Init.TIMode = 0;
    hspi2.Init.CRCCalculation = 0;
    hspi2.Init.CRCPolynomial = 0;
    hspi2.Init.CRCLength = 0;
    hspi2.Init.NSSPMode = 0;
}
