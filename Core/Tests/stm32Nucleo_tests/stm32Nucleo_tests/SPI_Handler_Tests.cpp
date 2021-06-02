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

using ::testing::_;
using ::testing::Return;

TEST(SpiHandlerTests, Test_HAL_GPIO_Config)
{
    Mock_HAL_GPIO mockGpio;
    Mock_HAL_SPI mockSpi;
    Mock_HAL_RCC mockRcc;
    Mock_HAL_NVIC mockNvic;

    const IRQn_Type spiIRQ_ID = SPI2_IRQn;
    const uint8_t spiIRQ_prio = 1U;
    const uint8_t spiIRQ_subPrio = 0U;

    const uint16_t SPI_SCLK_PIN = GPIO_PIN_13;
    const uint16_t SPI_MISO_PIN = GPIO_PIN_14;
    const uint16_t SPI_MOSI_PIN = GPIO_PIN_15;


    STUB_Reset_SpiConfigHandle();

    GPIO_InitTypeDef gpioConfigRet = mockGpio.getGpioConfig();

    EXPECT_CALL(mockRcc, __HAL_RCC_SPI2_CLK_ENABLE);
    EXPECT_CALL(mockRcc, __HAL_RCC_GPIOB_CLK_ENABLE);
    EXPECT_CALL(mockGpio, HAL_GPIO_Init(GPIOB, _));
    EXPECT_CALL(mockNvic, 
        HAL_NVIC_SetPriority(spiIRQ_ID, spiIRQ_prio, spiIRQ_subPrio));
    EXPECT_CALL(mockNvic, HAL_NVIC_EnableIRQ(spiIRQ_ID));
    EXPECT_CALL(mockSpi, HAL_SPI_Init(&hspi2)).WillOnce(Return(HAL_OK));

    EXPECT_EQ(true, SPI_Config());

    EXPECT_EQ(SPI_INIT, SPI_getState());

    gpioConfigRet = mockGpio.getGpioConfig();

    EXPECT_EQ(gpioConfigRet.Pin, SPI_SCLK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);
    EXPECT_EQ(gpioConfigRet.Mode, GPIO_MODE_AF_PP);
    EXPECT_EQ(gpioConfigRet.Pull, GPIO_NOPULL);
    EXPECT_EQ(gpioConfigRet.Speed, GPIO_SPEED_FREQ_HIGH);
    EXPECT_EQ(gpioConfigRet.Alternate, GPIO_AF0_SPI2);

    EXPECT_EQ(hspi2.Instance, SPI2);
    EXPECT_EQ(hspi2.Init.Mode, SPI_MODE_SLAVE);
    EXPECT_EQ(hspi2.Init.Direction, SPI_DIRECTION_2LINES);
    EXPECT_EQ(hspi2.Init.DataSize, SPI_DATASIZE_8BIT);
    EXPECT_EQ(hspi2.Init.CLKPolarity, SPI_POLARITY_LOW);
    EXPECT_EQ(hspi2.Init.CLKPhase, SPI_PHASE_1EDGE);
    EXPECT_EQ(hspi2.Init.NSS, SPI_NSS_HARD_INPUT);
    EXPECT_EQ(hspi2.Init.BaudRatePrescaler, SPI_BAUDRATEPRESCALER_32);// 1.5MHz
    EXPECT_EQ(hspi2.Init.FirstBit, SPI_FIRSTBIT_MSB);
    EXPECT_EQ(hspi2.Init.TIMode, SPI_TIMODE_DISABLE);
    EXPECT_EQ(hspi2.Init.CRCCalculation, SPI_CRCCALCULATION_DISABLE);
    EXPECT_EQ(hspi2.Init.CRCPolynomial, 7);
    EXPECT_EQ(hspi2.Init.CRCLength, SPI_CRC_LENGTH_DATASIZE);
    EXPECT_EQ(hspi2.Init.NSSPMode, SPI_NSS_PULSE_ENABLE);
}