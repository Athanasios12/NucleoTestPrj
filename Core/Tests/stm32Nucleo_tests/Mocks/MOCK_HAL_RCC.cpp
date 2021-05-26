#include "Mock_HAL_RCC.h"

static Mock_HAL_RCC *MockInstance = nullptr;

Mock_HAL_RCC::Mock_HAL_RCC()
{
    MockInstance = this;
}

Mock_HAL_RCC::~Mock_HAL_RCC()
{
    MockInstance = nullptr;
}

void __HAL_RCC_SPI2_CLK_ENABLE()
{
    MockInstance->__HAL_RCC_SPI2_CLK_ENABLE();
}

void __HAL_RCC_GPIOB_CLK_ENABLE()
{
    MockInstance->__HAL_RCC_GPIOB_CLK_ENABLE();
}