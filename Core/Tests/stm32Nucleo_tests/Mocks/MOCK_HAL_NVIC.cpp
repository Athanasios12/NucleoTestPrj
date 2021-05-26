#include "Mock_HAL_NVIC.h"

static Mock_HAL_NVIC *MockInstance = nullptr;

Mock_HAL_NVIC::Mock_HAL_NVIC()
{
    MockInstance = this;
}

Mock_HAL_NVIC::~Mock_HAL_NVIC()
{
    MockInstance = nullptr;
}

void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{
    MockInstance->HAL_NVIC_SetPriority(IRQn, PreemptPriority, SubPriority);
}

void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)
{
    MockInstance->HAL_NVIC_EnableIRQ(IRQn);
}