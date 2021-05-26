#ifndef MOCK_HAL_NVIC_H
#define MOCK_HAL_NVIC_H
#include <gmock/gmock.h>
extern "C"
{
#include "stm32Hal_NVIC_Stubs.h"
}

class Mock_HAL_NVIC
{
public:
    Mock_HAL_NVIC();
    virtual ~Mock_HAL_NVIC();

    MOCK_METHOD(void, HAL_NVIC_SetPriority, (IRQn_Type , uint32_t , uint32_t ));
    MOCK_METHOD(void, HAL_NVIC_EnableIRQ, (IRQn_Type));
};

#endif //MOCK_HAL_NVIC_H