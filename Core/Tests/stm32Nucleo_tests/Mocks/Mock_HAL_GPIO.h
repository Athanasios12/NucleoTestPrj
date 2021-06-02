#ifndef MOCK_HAL_GPIO_H
#define MOCK_HAL_GPIO_H
#include <gmock/gmock.h>
extern "C"
{
#include "stm32Hal_GPIO_Stubs.h"
}

class Mock_HAL_GPIO
{
public:
    Mock_HAL_GPIO();
    virtual ~Mock_HAL_GPIO();

    MOCK_METHOD(void, HAL_GPIO_Init, (GPIO_TypeDef *, GPIO_InitTypeDef *));

    GPIO_InitTypeDef getGpioConfig() const;
};

#endif //MOCK_HAL_GPIO_H