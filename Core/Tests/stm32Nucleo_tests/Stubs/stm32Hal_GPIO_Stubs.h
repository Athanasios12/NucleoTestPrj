#ifndef HAL_GPIO_STUBS_H
#define HAL_GPIO_STUBS_H
#include <stdint.h>
#include "common.h"


#define GPIO_PIN_0                 ((uint16_t)0x0001U)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002U)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004U)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008U)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010U)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020U)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040U)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080U)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100U)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200U)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400U)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800U)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000U)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000U)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000U)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000U)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFFU)  /* All pins selected */

#define GPIO_PIN_MASK              (0x0000FFFFU) /* PIN mask for assert test */

/** @defgroup GPIO_mode GPIO mode
  * @brief GPIO Configuration Mode
  *        Elements values convention: 0xX0yz00YZ
  *           - X  : GPIO mode or EXTI Mode
  *           - y  : External IT or Event trigger detection
  *           - z  : IO configuration on External IT or Event
  *           - Y  : Output type (Push Pull or Open Drain)
  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
  * @{
  */
#define  GPIO_MODE_INPUT                        (0x00000000U)   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    (0x00000001U)   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    (0x00000011U)   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        (0x00000002U)   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        (0x00000012U)   /*!< Alternate Function Open Drain Mode    */
#define  GPIO_MODE_ANALOG                       (0x00000003U)   /*!< Analog Mode  */  
#define  GPIO_MODE_IT_RISING                    (0x10110000U)   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   (0x10210000U)   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            (0x10310000U)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
#define  GPIO_MODE_EVT_RISING                   (0x10120000U)   /*!< External Event Mode with Rising edge trigger detection               */
#define  GPIO_MODE_EVT_FALLING                  (0x10220000U)   /*!< External Event Mode with Falling edge trigger detection              */
#define  GPIO_MODE_EVT_RISING_FALLING           (0x10320000U)   /*!< External Event Mode with Rising/Falling edge trigger detection       */

/** @defgroup GPIO_speed GPIO speed
  * @brief GPIO Output Maximum frequency
  * @{
  */  
#define  GPIO_SPEED_FREQ_LOW      (0x00000000U)  /*!< range up to 2 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_MEDIUM   (0x00000001U)  /*!< range  4 MHz to 10 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_HIGH     (0x00000003U)  /*!< range 10 MHz to 50 MHz, please refer to the product datasheet */
/**
* @}
*/

/** @defgroup GPIO_pull GPIO pull
* @brief GPIO Pull-Up or Pull-Down Activation
* @{
*/  
#define  GPIO_NOPULL        (0x00000000U)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        (0x00000001U)   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      (0x00000002U)   /*!< Pull-down activation                */
/** @defgroup GPIO_Exported_Types GPIO Exported Types
* @{
*/

/*---------------------------------- STM32F030x8 -------------------------------------------*/
/* AF 0 */
#define GPIO_AF0_EVENTOUT     ((uint8_t)0x00U)  /*!< AF0: EVENTOUT Alternate Function mapping  */
#define GPIO_AF0_MCO          ((uint8_t)0x00U)  /*!< AF0: MCO Alternate Function mapping       */
#define GPIO_AF0_SPI1         ((uint8_t)0x00U)  /*!< AF0: SPI1 Alternate Function mapping      */
#define GPIO_AF0_SPI2         ((uint8_t)0x00U)  /*!< AF0: SPI2 Alternate Function mapping      */
#define GPIO_AF0_TIM15        ((uint8_t)0x00U)  /*!< AF0: TIM15 Alternate Function mapping     */
#define GPIO_AF0_TIM17        ((uint8_t)0x00U)  /*!< AF0: TIM17 Alternate Function mapping     */
#define GPIO_AF0_SWDIO        ((uint8_t)0x00U)  /*!< AF0: SWDIO Alternate Function mapping     */
#define GPIO_AF0_SWCLK        ((uint8_t)0x00U)  /*!< AF0: SWCLK Alternate Function mapping     */
#define GPIO_AF0_TIM14        ((uint8_t)0x00U)  /*!< AF0: TIM14 Alternate Function mapping     */
#define GPIO_AF0_USART1       ((uint8_t)0x00U)  /*!< AF0: USART1 Alternate Function mapping    */
#define GPIO_AF0_IR           ((uint8_t)0x00U)  /*!< AF0: IR Alternate Function mapping        */
#define GPIO_AF0_TIM3         ((uint8_t)0x00U)  /*!< AF0: TIM3 Alternate Function mapping      */

/* AF 1 */
#define GPIO_AF1_TIM3         ((uint8_t)0x01U)  /*!< AF1: TIM3 Alternate Function mapping      */
#define GPIO_AF1_TIM15        ((uint8_t)0x01U)  /*!< AF1: TIM15 Alternate Function mapping     */
#define GPIO_AF1_USART1       ((uint8_t)0x01U)  /*!< AF1: USART1 Alternate Function mapping    */
#define GPIO_AF1_USART2       ((uint8_t)0x01U)  /*!< AF1: USART2 Alternate Function mapping    */
#define GPIO_AF1_EVENTOUT     ((uint8_t)0x01U)  /*!< AF1: EVENTOUT Alternate Function mapping  */
#define GPIO_AF1_I2C1         ((uint8_t)0x01U)  /*!< AF1: I2C1 Alternate Function mapping      */
#define GPIO_AF1_I2C2         ((uint8_t)0x01U)  /*!< AF1: I2C2 Alternate Function mapping      */
#define GPIO_AF1_IR           ((uint8_t)0x01U)  /*!< AF1: IR Alternate Function mapping        */

/* AF 2 */
#define GPIO_AF2_TIM1         ((uint8_t)0x02U)  /*!< AF2: TIM1 Alternate Function mapping      */
#define GPIO_AF2_TIM16        ((uint8_t)0x02U)  /*!< AF2: TIM16 Alternate Function mapping     */
#define GPIO_AF2_TIM17        ((uint8_t)0x02U)  /*!< AF2: TIM17 Alternate Function mapping     */
#define GPIO_AF2_EVENTOUT     ((uint8_t)0x02U)  /*!< AF2: EVENTOUT Alternate Function mapping  */

/* AF 3 */
#define GPIO_AF3_EVENTOUT     ((uint8_t)0x03U)  /*!< AF3: EVENTOUT Alternate Function mapping  */
#define GPIO_AF3_I2C1         ((uint8_t)0x03U)  /*!< AF3: I2C1 Alternate Function mapping      */
#define GPIO_AF3_TIM15        ((uint8_t)0x03U)  /*!< AF3: TIM15 Alternate Function mapping     */

/* AF 4 */
#define GPIO_AF4_TIM14        ((uint8_t)0x04U)  /*!< AF4: TIM14 Alternate Function mapping     */

/* AF 5 */
#define GPIO_AF5_TIM16        ((uint8_t)0x05U)  /*!< AF5: TIM16 Alternate Function mapping     */
#define GPIO_AF5_TIM17        ((uint8_t)0x05U)  /*!< AF5: TIM17 Alternate Function mapping     */

/* AF 6 */
#define GPIO_AF6_EVENTOUT     ((uint8_t)0x06U)  /*!< AF6: EVENTOUT Alternate Function mapping  */

#define IS_GPIO_AF(AF)        ((AF) <= (uint8_t)0x06U)

/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800UL)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000C00UL)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400UL)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
/**
* @brief   GPIO Init structure definition
*/
typedef struct
{
    uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                             This parameter can be any value of @ref GPIO_pins */

    uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                             This parameter can be a value of @ref GPIO_mode */

    uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                             This parameter can be a value of @ref GPIO_pull */

    uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                             This parameter can be a value of @ref GPIO_speed */

    uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins
                              This parameter can be a value of @ref GPIOEx_Alternate_function_selection */
}GPIO_InitTypeDef;

typedef struct
{
    uint32_t MODER;        /*!< GPIO port mode register,                     Address offset: 0x00      */
    uint32_t OTYPER;       /*!< GPIO port output type register,              Address offset: 0x04      */
    uint32_t OSPEEDR;      /*!< GPIO port output speed register,             Address offset: 0x08      */
    uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C      */
    uint32_t IDR;          /*!< GPIO port input data register,               Address offset: 0x10      */
    uint32_t ODR;          /*!< GPIO port output data register,              Address offset: 0x14      */
    uint32_t BSRR;         /*!< GPIO port bit set/reset register,      Address offset: 0x1A */
    uint32_t LCKR;         /*!< GPIO port configuration lock register,       Address offset: 0x1C      */
    uint32_t AFR[2];       /*!< GPIO alternate function low register,  Address offset: 0x20-0x24 */
    uint32_t BRR;          /*!< GPIO bit reset register,                     Address offset: 0x28      */
} GPIO_TypeDef;

void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);

#endif //HAL_GPIO_STUBS_H
