#ifndef HAL_NVIC_STUBS_H
#define HAL_NVIC_STUBS_H
#include "common.h"
#include <stdint.h>

/*!< Interrupt Number Definition */
typedef enum
{
    /******  Cortex-M0 Processor Exceptions Numbers **************************************************************/
    NonMaskableInt_IRQn = -14,    /*!< 2 Non Maskable Interrupt                                        */
    HardFault_IRQn = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                                */
    SVC_IRQn = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                                  */
    PendSV_IRQn = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                                  */
    SysTick_IRQn = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                              */

  /******  STM32F0 specific Interrupt Numbers ******************************************************************/
  WWDG_IRQn = 0,      /*!< Window WatchDog Interrupt                               */
  RTC_IRQn = 2,      /*!< RTC Interrupt through EXTI Lines 17, 19 and 20                  */
  FLASH_IRQn = 3,      /*!< FLASH global Interrupt                                          */
  RCC_IRQn = 4,      /*!< RCC global Interrupt                                            */
  EXTI0_1_IRQn = 5,      /*!< EXTI Line 0 and 1 Interrupt                                     */
  EXTI2_3_IRQn = 6,      /*!< EXTI Line 2 and 3 Interrupt                                     */
  EXTI4_15_IRQn = 7,      /*!< EXTI Line 4 to 15 Interrupt                                     */
  DMA1_Channel1_IRQn = 9,      /*!< DMA1 Channel 1 Interrupt                                        */
  DMA1_Channel2_3_IRQn = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupt                          */
  DMA1_Channel4_5_IRQn = 11,     /*!< DMA1 Channel 4 and Channel 5 Interrupt                          */
  ADC1_IRQn = 12,     /*!< ADC1 Interrupt                                                  */
  TIM1_BRK_UP_TRG_COM_IRQn = 13,     /*!< TIM1 Break, Update, Trigger and Commutation Interrupt           */
  TIM1_CC_IRQn = 14,     /*!< TIM1 Capture Compare Interrupt                                  */
  TIM3_IRQn = 16,     /*!< TIM3 global Interrupt                                           */
  TIM6_IRQn = 17,     /*!< TIM6 global Interrupt                                           */
  TIM14_IRQn = 19,     /*!< TIM14 global Interrupt                                          */
  TIM15_IRQn = 20,     /*!< TIM15 global Interrupt                                          */
  TIM16_IRQn = 21,     /*!< TIM16 global Interrupt                                          */
  TIM17_IRQn = 22,     /*!< TIM17 global Interrupt                                          */
  I2C1_IRQn = 23,     /*!< I2C1 Event Interrupt                                            */
  I2C2_IRQn = 24,     /*!< I2C2 Event Interrupt                                            */
  SPI1_IRQn = 25,     /*!< SPI1 global Interrupt                                           */
  SPI2_IRQn = 26,     /*!< SPI2 global Interrupt                                           */
  USART1_IRQn = 27,      /*!< USART1 global Interrupt                                        */
  USART2_IRQn = 28      /*!< USART2 global Interrupt                                         */
} IRQn_Type;

void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);

#endif // HAL_SPI_STUBS_H