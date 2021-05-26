#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "Mock_HAL_GPIO.h"
#include "Mock_HAL_NVIC.h"
#include "Mock_HAL_RCC.h"
#include "Mock_HAL_SPI.h"
extern "C"
{
#include "SPI_Handler.h"
}

TEST(SpiHandlerTests, Test_HAL_GPIO_Config)
{
    Mock_HAL_GPIO mockGpio;
    Mock_HAL_SPI mockSpi;
    Mock_HAL_RCC mockRcc;
    Mock_HAL_NVIC mockNvic;

    EXPECT_CALL(mockRcc, __HAL_RCC_SPI2_CLK_ENABLE);
    EXPECT_CALL(mockRcc, __HAL_RCC_GPIOB_CLK_ENABLE);
    EXPECT_CALL(mockGpio, HAL_GPIO_Init);
    EXPECT_CALL(mockNvic, HAL_NVIC_SetPriority);
    EXPECT_CALL(mockNvic, HAL_NVIC_EnableIRQ);
    EXPECT_CALL(mockSpi, HAL_SPI_Init);

    SPI_Config();

    EXPECT_EQ(1000, 1000);
}