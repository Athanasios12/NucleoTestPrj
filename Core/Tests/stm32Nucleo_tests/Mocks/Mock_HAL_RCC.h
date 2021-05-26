#ifndef MOCK_HAL_RCC_H
#define MOCK_HAL_RCC_H
#include <gmock/gmock.h>
extern "C"
{
#include "stm32Hal_RCC_Stubs.h"
}

class Mock_HAL_RCC
{
public:
    Mock_HAL_RCC();
    virtual ~Mock_HAL_RCC();

    MOCK_METHOD(void, __HAL_RCC_SPI2_CLK_ENABLE, ());
    MOCK_METHOD(void, __HAL_RCC_GPIOB_CLK_ENABLE, ());
};

#endif //MOCK_HAL_RCC_H