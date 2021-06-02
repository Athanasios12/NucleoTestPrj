#include "Mock_HAL_GPIO.h"

static Mock_HAL_GPIO *MockInstance = nullptr;
static GPIO_InitTypeDef gpioConfig = { 0 };

Mock_HAL_GPIO::Mock_HAL_GPIO()
{
    MockInstance = this;
}

Mock_HAL_GPIO::~Mock_HAL_GPIO()
{
    MockInstance = nullptr;
    gpioConfig.Pin = 0;
    gpioConfig.Mode = 0;
    gpioConfig.Speed = 0;
    gpioConfig.Pull = 0;
}

void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    if (MockInstance)
    {
        gpioConfig = *GPIO_Init;
        MockInstance->HAL_GPIO_Init(GPIOx, GPIO_Init);
    }
}

GPIO_InitTypeDef Mock_HAL_GPIO::getGpioConfig() const
{
    return gpioConfig;
}