#include "Mock_HAL_GPIO.h"

static Mock_HAL_GPIO *MockInstance = nullptr;

Mock_HAL_GPIO::Mock_HAL_GPIO()
{
    MockInstance = this;
}

Mock_HAL_GPIO::~Mock_HAL_GPIO()
{
    MockInstance = nullptr;
}

void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    if (MockInstance)
    {
        MockInstance->HAL_GPIO_Init(GPIOx, GPIO_Init);
    }
}