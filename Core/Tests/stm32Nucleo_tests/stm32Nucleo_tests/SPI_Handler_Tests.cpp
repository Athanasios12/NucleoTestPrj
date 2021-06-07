#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "Mock_HAL_GPIO.h"
#include "Mock_HAL_NVIC.h"
#include "Mock_HAL_RCC.h"
#include "Mock_HAL_SPI.h"
extern "C"
{
#include "SPI_Handler.h"
#include "stm32f0xx_hal.h"
}

using ::testing::_;
using ::testing::Return;
using ::testing::SetArrayArgument;
using ::testing::ElementsAreArray;
using ::testing::Args;

MATCHER_P2(HasBytes, bytes, size, "")
{
    return (0 == memcmp(arg, bytes, size));
}

TEST(SpiHandlerTests, Test_SPI_Config_Success)
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


    mockSpi.resetSpiConfig();
    EXPECT_EQ(hspi2.Instance, NoDevice);

    mockGpio.getGpioConfig();

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

TEST(SpiHandlerTests, Test_SPI_Init_Success)
{
    Mock_HAL_SPI mockSpi;

    EXPECT_EQ(SPI_INIT, SPI_getState());
    EXPECT_EQ(false, SPI_CommEstablished());

    char rxBuffer[SPI_RX_BUFF_SIZE];
    const char initMsg[] = "RPI_INI";
    strncpy_s(rxBuffer, SPI_RX_BUFF_SIZE, initMsg, strlen(initMsg));

    EXPECT_CALL(mockSpi, HAL_SPI_Receive_IT(&hspi2, _, SPI_RX_BUFF_SIZE))
        .WillOnce(DoAll(SetArrayArgument<1>(rxBuffer, rxBuffer + SPI_RX_BUFF_SIZE - 1), Return(HAL_OK)));

    SPI_Init();

    EXPECT_EQ(SPI_RX, SPI_getState());    
}

TEST(SpiHandlerTests, Test_SPI_CommSM_EstablishComm)
{
    Mock_HAL_SPI mockSpi;
    uint8_t txBuffer[SPI_TX_BUFF_SIZE] = { 0 };
    const char responseMsg[] = "STM_ACK";
    strncpy_s((char*)txBuffer, SPI_RX_BUFF_SIZE, responseMsg, strlen(responseMsg));

    EXPECT_EQ(SPI_RX, SPI_getState());
    EXPECT_EQ(false, SPI_CommEstablished());

    //simulate IRQHandler calling the callback function    
    EXPECT_EQ(false, spi_rx_done);
    HAL_SPI_RxCpltCallback(&hspi2);
    EXPECT_EQ(true, spi_rx_done);

    EXPECT_CALL(mockSpi, HAL_SPI_Transmit_IT(&hspi2,
        HasBytes(txBuffer, strlen(responseMsg)), SPI_TX_BUFF_SIZE));

    SPI_CommSM();

    EXPECT_EQ(SPI_TX, SPI_getState());
    EXPECT_EQ(true, SPI_CommEstablished());
}