#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "Mock_HAL_GPIO.h"
#include "Mock_HAL_NVIC.h"
#include "Mock_HAL_RCC.h"
#include "Mock_HAL_SPI.h"
extern "C"
{
//#include "dht11.h"
#include "stm32f0xx_hal.h"
}

using ::testing::_;
using ::testing::Return;
using ::testing::SetArrayArgument;
using ::testing::ElementsAreArray;
using ::testing::Args;


TEST(Dht11Tests, Test1)
{
    EXPECT_EQ(0, 0);
}