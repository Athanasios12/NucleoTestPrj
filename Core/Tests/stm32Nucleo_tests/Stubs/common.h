#pragma once
#define PERIPH_BASE           0x40000000UL              /*!< Peripheral base address in the alias region */

/*!< Peripheral memory map */
#define APBPERIPH_BASE        PERIPH_BASE
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000UL)


