#include "stm32Hal_SPI_Stubs.h"

SPI_HandleTypeDef hspi2;

void STUB_Reset_SpiConfigHandle()
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