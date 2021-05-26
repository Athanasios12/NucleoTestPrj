#ifndef HAL_SPI_STUBS_H
#define HAL_SPI_STUBS_H
#include <stdint.h>
#include "common.h"

/*****************************************************************************/
/*                                                                           */
/*                        Serial Peripheral Interface (SPI)                  */
/*                                                                           */
/*****************************************************************************/

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F0 serie)
 */
 /* Note: No specific macro feature on this device */

 /*******************  Bit definition for SPI_CR1 register  *******************/
#define SPI_CR1_CPHA_Pos            (0U)                                       
#define SPI_CR1_CPHA_Msk            (0x1UL << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk                           /*!< Clock Phase */
#define SPI_CR1_CPOL_Pos            (1U)                                       
#define SPI_CR1_CPOL_Msk            (0x1UL << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk                           /*!< Clock Polarity */
#define SPI_CR1_MSTR_Pos            (2U)                                       
#define SPI_CR1_MSTR_Msk            (0x1UL << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk                           /*!< Master Selection */
#define SPI_CR1_BR_Pos              (3U)                                       
#define SPI_CR1_BR_Msk              (0x7UL << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk                             /*!< BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                (0x1UL << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2UL << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4UL << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */
#define SPI_CR1_SPE_Pos             (6U)                                       
#define SPI_CR1_SPE_Msk             (0x1UL << SPI_CR1_SPE_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_SPE                 SPI_CR1_SPE_Msk                            /*!< SPI Enable */
#define SPI_CR1_LSBFIRST_Pos        (7U)                                       
#define SPI_CR1_LSBFIRST_Msk        (0x1UL << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk                       /*!< Frame Format */
#define SPI_CR1_SSI_Pos             (8U)                                       
#define SPI_CR1_SSI_Msk             (0x1UL << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk                            /*!< Internal slave select */
#define SPI_CR1_SSM_Pos             (9U)                                       
#define SPI_CR1_SSM_Msk             (0x1UL << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk                            /*!< Software slave management */
#define SPI_CR1_RXONLY_Pos          (10U)                                      
#define SPI_CR1_RXONLY_Msk          (0x1UL << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk                         /*!< Receive only */
#define SPI_CR1_CRCL_Pos            (11U)                                      
#define SPI_CR1_CRCL_Msk            (0x1UL << SPI_CR1_CRCL_Pos)                 /*!< 0x00000800 */
#define SPI_CR1_CRCL                SPI_CR1_CRCL_Msk                           /*!< CRC Length */
#define SPI_CR1_CRCNEXT_Pos         (12U)                                      
#define SPI_CR1_CRCNEXT_Msk         (0x1UL << SPI_CR1_CRCNEXT_Pos)              /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT             SPI_CR1_CRCNEXT_Msk                        /*!< Transmit CRC next */
#define SPI_CR1_CRCEN_Pos           (13U)                                      
#define SPI_CR1_CRCEN_Msk           (0x1UL << SPI_CR1_CRCEN_Pos)                /*!< 0x00002000 */
#define SPI_CR1_CRCEN               SPI_CR1_CRCEN_Msk                          /*!< Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE_Pos          (14U)                                      
#define SPI_CR1_BIDIOE_Msk          (0x1UL << SPI_CR1_BIDIOE_Pos)               /*!< 0x00004000 */
#define SPI_CR1_BIDIOE              SPI_CR1_BIDIOE_Msk                         /*!< Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos        (15U)                                      
#define SPI_CR1_BIDIMODE_Msk        (0x1UL << SPI_CR1_BIDIMODE_Pos)             /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE            SPI_CR1_BIDIMODE_Msk                       /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  *******************/
#define SPI_CR2_RXDMAEN_Pos         (0U)                                       
#define SPI_CR2_RXDMAEN_Msk         (0x1UL << SPI_CR2_RXDMAEN_Pos)              /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN             SPI_CR2_RXDMAEN_Msk                        /*!< Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN_Pos         (1U)                                       
#define SPI_CR2_TXDMAEN_Msk         (0x1UL << SPI_CR2_TXDMAEN_Pos)              /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN             SPI_CR2_TXDMAEN_Msk                        /*!< Tx Buffer DMA Enable */
#define SPI_CR2_SSOE_Pos            (2U)                                       
#define SPI_CR2_SSOE_Msk            (0x1UL << SPI_CR2_SSOE_Pos)                 /*!< 0x00000004 */
#define SPI_CR2_SSOE                SPI_CR2_SSOE_Msk                           /*!< SS Output Enable */
#define SPI_CR2_NSSP_Pos            (3U)                                       
#define SPI_CR2_NSSP_Msk            (0x1UL << SPI_CR2_NSSP_Pos)                 /*!< 0x00000008 */
#define SPI_CR2_NSSP                SPI_CR2_NSSP_Msk                           /*!< NSS pulse management Enable */
#define SPI_CR2_FRF_Pos             (4U)                                       
#define SPI_CR2_FRF_Msk             (0x1UL << SPI_CR2_FRF_Pos)                  /*!< 0x00000010 */
#define SPI_CR2_FRF                 SPI_CR2_FRF_Msk                            /*!< Frame Format Enable */
#define SPI_CR2_ERRIE_Pos           (5U)                                       
#define SPI_CR2_ERRIE_Msk           (0x1UL << SPI_CR2_ERRIE_Pos)                /*!< 0x00000020 */
#define SPI_CR2_ERRIE               SPI_CR2_ERRIE_Msk                          /*!< Error Interrupt Enable */
#define SPI_CR2_RXNEIE_Pos          (6U)                                       
#define SPI_CR2_RXNEIE_Msk          (0x1UL << SPI_CR2_RXNEIE_Pos)               /*!< 0x00000040 */
#define SPI_CR2_RXNEIE              SPI_CR2_RXNEIE_Msk                         /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos           (7U)                                       
#define SPI_CR2_TXEIE_Msk           (0x1UL << SPI_CR2_TXEIE_Pos)                /*!< 0x00000080 */
#define SPI_CR2_TXEIE               SPI_CR2_TXEIE_Msk                          /*!< Tx buffer Empty Interrupt Enable */
#define SPI_CR2_DS_Pos              (8U)                                       
#define SPI_CR2_DS_Msk              (0xFUL << SPI_CR2_DS_Pos)                   /*!< 0x00000F00 */
#define SPI_CR2_DS                  SPI_CR2_DS_Msk                             /*!< DS[3:0] Data Size */
#define SPI_CR2_DS_0                (0x1UL << SPI_CR2_DS_Pos)                   /*!< 0x00000100 */
#define SPI_CR2_DS_1                (0x2UL << SPI_CR2_DS_Pos)                   /*!< 0x00000200 */
#define SPI_CR2_DS_2                (0x4UL << SPI_CR2_DS_Pos)                   /*!< 0x00000400 */
#define SPI_CR2_DS_3                (0x8UL << SPI_CR2_DS_Pos)                   /*!< 0x00000800 */
#define SPI_CR2_FRXTH_Pos           (12U)                                      
#define SPI_CR2_FRXTH_Msk           (0x1UL << SPI_CR2_FRXTH_Pos)                /*!< 0x00001000 */
#define SPI_CR2_FRXTH               SPI_CR2_FRXTH_Msk                          /*!< FIFO reception Threshold */
#define SPI_CR2_LDMARX_Pos          (13U)                                      
#define SPI_CR2_LDMARX_Msk          (0x1UL << SPI_CR2_LDMARX_Pos)               /*!< 0x00002000 */
#define SPI_CR2_LDMARX              SPI_CR2_LDMARX_Msk                         /*!< Last DMA transfer for reception */
#define SPI_CR2_LDMATX_Pos          (14U)                                      
#define SPI_CR2_LDMATX_Msk          (0x1UL << SPI_CR2_LDMATX_Pos)               /*!< 0x00004000 */
#define SPI_CR2_LDMATX              SPI_CR2_LDMATX_Msk                         /*!< Last DMA transfer for transmission */

/********************  Bit definition for SPI_SR register  *******************/
#define SPI_SR_RXNE_Pos             (0U)                                       
#define SPI_SR_RXNE_Msk             (0x1UL << SPI_SR_RXNE_Pos)                  /*!< 0x00000001 */
#define SPI_SR_RXNE                 SPI_SR_RXNE_Msk                            /*!< Receive buffer Not Empty */
#define SPI_SR_TXE_Pos              (1U)                                       
#define SPI_SR_TXE_Msk              (0x1UL << SPI_SR_TXE_Pos)                   /*!< 0x00000002 */
#define SPI_SR_TXE                  SPI_SR_TXE_Msk                             /*!< Transmit buffer Empty */
#define SPI_SR_CRCERR_Pos           (4U)                                       
#define SPI_SR_CRCERR_Msk           (0x1UL << SPI_SR_CRCERR_Pos)                /*!< 0x00000010 */
#define SPI_SR_CRCERR               SPI_SR_CRCERR_Msk                          /*!< CRC Error flag */
#define SPI_SR_MODF_Pos             (5U)                                       
#define SPI_SR_MODF_Msk             (0x1UL << SPI_SR_MODF_Pos)                  /*!< 0x00000020 */
#define SPI_SR_MODF                 SPI_SR_MODF_Msk                            /*!< Mode fault */
#define SPI_SR_OVR_Pos              (6U)                                       
#define SPI_SR_OVR_Msk              (0x1UL << SPI_SR_OVR_Pos)                   /*!< 0x00000040 */
#define SPI_SR_OVR                  SPI_SR_OVR_Msk                             /*!< Overrun flag */
#define SPI_SR_BSY_Pos              (7U)                                       
#define SPI_SR_BSY_Msk              (0x1UL << SPI_SR_BSY_Pos)                   /*!< 0x00000080 */
#define SPI_SR_BSY                  SPI_SR_BSY_Msk                             /*!< Busy flag */
#define SPI_SR_FRE_Pos              (8U)                                       
#define SPI_SR_FRE_Msk              (0x1UL << SPI_SR_FRE_Pos)                   /*!< 0x00000100 */
#define SPI_SR_FRE                  SPI_SR_FRE_Msk                             /*!< TI frame format error */
#define SPI_SR_FRLVL_Pos            (9U)                                       
#define SPI_SR_FRLVL_Msk            (0x3UL << SPI_SR_FRLVL_Pos)                 /*!< 0x00000600 */
#define SPI_SR_FRLVL                SPI_SR_FRLVL_Msk                           /*!< FIFO Reception Level */
#define SPI_SR_FRLVL_0              (0x1UL << SPI_SR_FRLVL_Pos)                 /*!< 0x00000200 */
#define SPI_SR_FRLVL_1              (0x2UL << SPI_SR_FRLVL_Pos)                 /*!< 0x00000400 */
#define SPI_SR_FTLVL_Pos            (11U)                                      
#define SPI_SR_FTLVL_Msk            (0x3UL << SPI_SR_FTLVL_Pos)                 /*!< 0x00001800 */
#define SPI_SR_FTLVL                SPI_SR_FTLVL_Msk                           /*!< FIFO Transmission Level */
#define SPI_SR_FTLVL_0              (0x1UL << SPI_SR_FTLVL_Pos)                 /*!< 0x00000800 */
#define SPI_SR_FTLVL_1              (0x2UL << SPI_SR_FTLVL_Pos)                 /*!< 0x00001000 */

/********************  Bit definition for SPI_DR register  *******************/
#define SPI_DR_DR_Pos               (0U)                                       
#define SPI_DR_DR_Msk               (0xFFFFFFFFUL << SPI_DR_DR_Pos)             /*!< 0xFFFFFFFF */
#define SPI_DR_DR                   SPI_DR_DR_Msk                              /*!< Data Register */

/*******************  Bit definition for SPI_CRCPR register  *****************/
#define SPI_CRCPR_CRCPOLY_Pos       (0U)                                       
#define SPI_CRCPR_CRCPOLY_Msk       (0xFFFFFFFFUL << SPI_CRCPR_CRCPOLY_Pos)     /*!< 0xFFFFFFFF */
#define SPI_CRCPR_CRCPOLY           SPI_CRCPR_CRCPOLY_Msk                      /*!< CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  *****************/
#define SPI_RXCRCR_RXCRC_Pos        (0U)                                       
#define SPI_RXCRCR_RXCRC_Msk        (0xFFFFFFFFUL << SPI_RXCRCR_RXCRC_Pos)      /*!< 0xFFFFFFFF */
#define SPI_RXCRCR_RXCRC            SPI_RXCRCR_RXCRC_Msk                       /*!< Rx CRC Register */

/******************  Bit definition for SPI_TXCRCR register  *****************/
#define SPI_TXCRCR_TXCRC_Pos        (0U)                                       
#define SPI_TXCRCR_TXCRC_Msk        (0xFFFFFFFFUL << SPI_TXCRCR_TXCRC_Pos)      /*!< 0xFFFFFFFF */
#define SPI_TXCRCR_TXCRC            SPI_TXCRCR_TXCRC_Msk                       /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  ****************/
#define SPI_I2SCFGR_I2SMOD_Pos      (11U)                                      
#define SPI_I2SCFGR_I2SMOD_Msk      (0x1UL << SPI_I2SCFGR_I2SMOD_Pos)           /*!< 0x00000800 */
#define SPI_I2SCFGR_I2SMOD          SPI_I2SCFGR_I2SMOD_Msk                     /*!< Keep for compatibility */

/** @defgroup SPI_Mode SPI Mode
  * @{
  */
#define SPI_MODE_SLAVE                  (0x00000000U)
#define SPI_MODE_MASTER                 (SPI_CR1_MSTR | SPI_CR1_SSI)
/**
* @}
*/

/** @defgroup SPI_Direction SPI Direction Mode
* @{
*/
#define SPI_DIRECTION_2LINES            (0x00000000U)
#define SPI_DIRECTION_2LINES_RXONLY     SPI_CR1_RXONLY
#define SPI_DIRECTION_1LINE             SPI_CR1_BIDIMODE
/**
* @}
*/

/** @defgroup SPI_Data_Size SPI Data Size
* @{
*/
#define SPI_DATASIZE_4BIT               (0x00000300U)
#define SPI_DATASIZE_5BIT               (0x00000400U)
#define SPI_DATASIZE_6BIT               (0x00000500U)
#define SPI_DATASIZE_7BIT               (0x00000600U)
#define SPI_DATASIZE_8BIT               (0x00000700U)
#define SPI_DATASIZE_9BIT               (0x00000800U)
#define SPI_DATASIZE_10BIT              (0x00000900U)
#define SPI_DATASIZE_11BIT              (0x00000A00U)
#define SPI_DATASIZE_12BIT              (0x00000B00U)
#define SPI_DATASIZE_13BIT              (0x00000C00U)
#define SPI_DATASIZE_14BIT              (0x00000D00U)
#define SPI_DATASIZE_15BIT              (0x00000E00U)
#define SPI_DATASIZE_16BIT              (0x00000F00U)
/**
* @}
*/

/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
* @{
*/
#define SPI_POLARITY_LOW                (0x00000000U)
#define SPI_POLARITY_HIGH               SPI_CR1_CPOL
/**
* @}
*/

/** @defgroup SPI_Clock_Phase SPI Clock Phase
* @{
*/
#define SPI_PHASE_1EDGE                 (0x00000000U)
#define SPI_PHASE_2EDGE                 SPI_CR1_CPHA
/**
* @}
*/

/** @defgroup SPI_Slave_Select_management SPI Slave Select Management
* @{
*/
#define SPI_NSS_SOFT                    SPI_CR1_SSM
#define SPI_NSS_HARD_INPUT              (0x00000000U)
#define SPI_NSS_HARD_OUTPUT             (SPI_CR2_SSOE << 16U)
/**
* @}
*/

/** @defgroup SPI_NSSP_Mode SPI NSS Pulse Mode
* @{
*/
#define SPI_NSS_PULSE_ENABLE            SPI_CR2_NSSP
#define SPI_NSS_PULSE_DISABLE           (0x00000000U)
/**
* @}
*/

/** @defgroup SPI_BaudRate_Prescaler SPI BaudRate Prescaler
* @{
*/
#define SPI_BAUDRATEPRESCALER_2         (0x00000000U)
#define SPI_BAUDRATEPRESCALER_4         (SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_8         (SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_16        (SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_32        (SPI_CR1_BR_2)
#define SPI_BAUDRATEPRESCALER_64        (SPI_CR1_BR_2 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_128       (SPI_CR1_BR_2 | SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_256       (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)
/**
* @}
*/

/** @defgroup SPI_MSB_LSB_transmission SPI MSB LSB Transmission
* @{
*/
#define SPI_FIRSTBIT_MSB                (0x00000000U)
#define SPI_FIRSTBIT_LSB                SPI_CR1_LSBFIRST
/**
* @}
*/

/** @defgroup SPI_TI_mode SPI TI Mode
* @{
*/
#define SPI_TIMODE_DISABLE              (0x00000000U)
#define SPI_TIMODE_ENABLE               SPI_CR2_FRF
/**
* @}
*/

/** @defgroup SPI_CRC_Calculation SPI CRC Calculation
* @{
*/
#define SPI_CRCCALCULATION_DISABLE      (0x00000000U)
#define SPI_CRCCALCULATION_ENABLE       SPI_CR1_CRCEN
/**
* @}
*/

/** @defgroup SPI_CRC_length SPI CRC Length
* @{
* This parameter can be one of the following values:
*     SPI_CRC_LENGTH_DATASIZE: aligned with the data size
*     SPI_CRC_LENGTH_8BIT    : CRC 8bit
*     SPI_CRC_LENGTH_16BIT   : CRC 16bit
*/
#define SPI_CRC_LENGTH_DATASIZE         (0x00000000U)
#define SPI_CRC_LENGTH_8BIT             (0x00000001U)
#define SPI_CRC_LENGTH_16BIT            (0x00000002U)
/**
* @}
*/


typedef enum
{
    SPI1 = 0,
    SPI2
} SPI_Devices;

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/**
  * @brief  SPI Configuration Structure definition
  */
typedef struct
{
    uint32_t Mode;                /*!< Specifies the SPI operating mode.
                                       This parameter can be a value of @ref SPI_Mode */

    uint32_t Direction;           /*!< Specifies the SPI bidirectional mode state.
                                       This parameter can be a value of @ref SPI_Direction */

    uint32_t DataSize;            /*!< Specifies the SPI data size.
                                       This parameter can be a value of @ref SPI_Data_Size */

    uint32_t CLKPolarity;         /*!< Specifies the serial clock steady state.
                                       This parameter can be a value of @ref SPI_Clock_Polarity */

    uint32_t CLKPhase;            /*!< Specifies the clock active edge for the bit capture.
                                       This parameter can be a value of @ref SPI_Clock_Phase */

    uint32_t NSS;                 /*!< Specifies whether the NSS signal is managed by
                                       hardware (NSS pin) or by software using the SSI bit.
                                       This parameter can be a value of @ref SPI_Slave_Select_management */

    uint32_t BaudRatePrescaler;   /*!< Specifies the Baud Rate prescaler value which will be
                                       used to configure the transmit and receive SCK clock.
                                       This parameter can be a value of @ref SPI_BaudRate_Prescaler
                                       @note The communication clock is derived from the master
                                       clock. The slave clock does not need to be set. */

    uint32_t FirstBit;            /*!< Specifies whether data transfers start from MSB or LSB bit.
                                       This parameter can be a value of @ref SPI_MSB_LSB_transmission */

    uint32_t TIMode;              /*!< Specifies if the TI mode is enabled or not.
                                       This parameter can be a value of @ref SPI_TI_mode */

    uint32_t CRCCalculation;      /*!< Specifies if the CRC calculation is enabled or not.
                                       This parameter can be a value of @ref SPI_CRC_Calculation */

    uint32_t CRCPolynomial;       /*!< Specifies the polynomial used for the CRC calculation.
                                       This parameter must be an odd number between Min_Data = 1 and Max_Data = 65535 */

    uint32_t CRCLength;           /*!< Specifies the CRC Length used for the CRC calculation.
                                       CRC Length is only used with Data8 and Data16, not other data size
                                       This parameter can be a value of @ref SPI_CRC_length */

    uint32_t NSSPMode;            /*!< Specifies whether the NSSP signal is enabled or not .
                                       This parameter can be a value of @ref SPI_NSSP_Mode
                                       This mode is activated by the NSSP bit in the SPIx_CR2 register and
                                       it takes effect only if the SPI interface is configured as Motorola SPI
                                       master (FRF=0) with capture on the first edge (SPIx_CR1 CPHA = 0,
                                       CPOL setting is ignored).. */
} SPI_InitTypeDef;

typedef struct
{
    SPI_InitTypeDef Init;
    uint8_t Instance;
} SPI_HandleTypeDef;

/* I/O operation functions  ***************************************************/

HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
    uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef* hspi);

#endif //HAL_SPI_STUBS_H