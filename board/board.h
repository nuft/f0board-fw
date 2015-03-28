/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STMicroelectronics NUCLEO-F030R8 board.
 */

/*
 * Board identifier.
 */
#define BOARD_F0
#define BOARD_NAME                  "f0board RevA"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 * NOTE: HSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#define STM32_LSEDRV                (3 << 3)

#define STM32_HSECLK                16000000

/*
 * MCU type as defined in the ST header.
 */
#define STM32F042x6

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0
#define GPIOA_PIN1                  1
#define GPIOA_PIN2                  2
#define GPIOA_NRF_CE                3
#define GPIOA_SPI1_CS               4
#define GPIOA_SPI1_SCK              5
#define GPIOA_SPI1_MISO             6
#define GPIOA_SPI1_MOSI             7
#define GPIOA_PIN8                  8
#define GPIOA_UART1_TX              9
#define GPIOA_UART1_RX              10
#define GPIOA_OTG_FS_DM             11
#define GPIOA_OTG_FS_DP             12
#define GPIOA_SWDIO                 13
#define GPIOA_SWCLK                 14
#define GPIOA_PIN15                 15

#define GPIOB_MPU_INT               0
#define GPIOB_NRF_INT               1
#define GPIOB_LED                   2
#define GPIOB_PIN3                  3
#define GPIOB_PIN4                  4
#define GPIOB_PIN5                  5
#define GPIOB_I2C1_SCL              6
#define GPIOB_I2C1_SDA              7
#define GPIOB_CAN_RX                8
#define GPIOB_CAN_TX                9
#define GPIOB_PIN10                 10
#define GPIOB_PIN11                 11
#define GPIOB_PIN12                 12
#define GPIOB_PIN13                 13
#define GPIOB_PIN14                 14
#define GPIOB_PIN15                 15

#define GPIOC_PIN13                 13
#define GPIOC_PIN14                 14
#define GPIOC_PIN15                 15

#define GPIOF_OSC_IN                0
#define GPIOF_OSC_OUT               1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_10M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_40M(n)           (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << (((n) % 8) * 4))

/*
 * GPIOA setup:
 *
 * PA0  - PIN0                      (input pulldown).
 * PA1  - PIN1                      (input pulldown).
 * PA2  - PIN2                      (input pulldown).
 * PA3  - NRF_CE                    (output low).
 * PA4  - SPI1_CS                   (output high).
 * PA5  - SPI1_SCK                  (alternate 0).
 * PA6  - SPI1_MISO                 (alternate 0).
 * PA7  - SPI1_MOSI                 (alternate 0).
 * PA8  - PIN8                      (input pulldown).
 * PA9  - UART1_TX                  (alternate 1).
 * PA10 - UART1_RX                  (alternate 1).
 * PA11 - OTG_FS_DM                 (alternate 10).
 * PA12 - OTG_FS_DP                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - PIN15                     (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN2) |           \
                                     PIN_MODE_OUTPUT(GPIOA_NRF_CE) |        \
                                     PIN_MODE_OUTPUT(GPIOA_SPI1_CS) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI) |  \
                                     PIN_MODE_INPUT(GPIOA_PIN8) |           \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_TX) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_RX) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_NRF_CE) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_CS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_40M(GPIOA_PIN0) |           \
                                     PIN_OSPEED_40M(GPIOA_PIN1) |           \
                                     PIN_OSPEED_40M(GPIOA_PIN2) |           \
                                     PIN_OSPEED_40M(GPIOA_NRF_CE) |         \
                                     PIN_OSPEED_40M(GPIOA_SPI1_CS) |        \
                                     PIN_OSPEED_40M(GPIOA_SPI1_SCK) |       \
                                     PIN_OSPEED_40M(GPIOA_SPI1_MISO) |      \
                                     PIN_OSPEED_40M(GPIOA_SPI1_MOSI) |      \
                                     PIN_OSPEED_40M(GPIOA_PIN8) |           \
                                     PIN_OSPEED_40M(GPIOA_UART1_TX) |       \
                                     PIN_OSPEED_40M(GPIOA_UART1_RX) |       \
                                     PIN_OSPEED_40M(GPIOA_OTG_FS_DM) |      \
                                     PIN_OSPEED_40M(GPIOA_OTG_FS_DP) |      \
                                     PIN_OSPEED_40M(GPIOA_SWDIO) |          \
                                     PIN_OSPEED_40M(GPIOA_SWCLK) |          \
                                     PIN_OSPEED_40M(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOA_PIN0) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN1) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_NRF_CE) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_CS) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_SCK) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MISO) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MOSI) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_TX) |   \
                                     PIN_PUPDR_PULLUP(GPIOA_UART1_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_PIN0) |              \
                                     PIN_ODR_LOW(GPIOA_PIN1) |              \
                                     PIN_ODR_LOW(GPIOA_PIN2) |              \
                                     PIN_ODR_LOW(GPIOA_NRF_CE) |            \
                                     PIN_ODR_HIGH(GPIOA_SPI1_CS) |          \
                                     PIN_ODR_LOW(GPIOA_SPI1_SCK) |          \
                                     PIN_ODR_LOW(GPIOA_SPI1_MISO) |         \
                                     PIN_ODR_LOW(GPIOA_SPI1_MOSI) |         \
                                     PIN_ODR_LOW(GPIOA_PIN8) |              \
                                     PIN_ODR_LOW(GPIOA_UART1_TX) |          \
                                     PIN_ODR_LOW(GPIOA_UART1_RX) |          \
                                     PIN_ODR_LOW(GPIOA_OTG_FS_DM) |         \
                                     PIN_ODR_LOW(GPIOA_OTG_FS_DP) |         \
                                     PIN_ODR_LOW(GPIOA_SWDIO) |             \
                                     PIN_ODR_LOW(GPIOA_SWCLK) |             \
                                     PIN_ODR_LOW(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOA_NRF_CE, 0) |         \
                                     PIN_AFIO_AF(GPIOA_SPI1_CS, 0) |        \
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK, 0) |       \
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO, 0) |      \
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOA_UART1_TX, 1) |       \
                                     PIN_AFIO_AF(GPIOA_UART1_RX, 1) |       \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |     \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) |     \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) |          \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) |          \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0))

/*
 * GPIOB setup:
 *
 * PB0  - MPU_INT                   (input floating).
 * PB1  - NRF_INT                   (input floating).
 * PB2  - LED                       (output low, max).
 * PB3  - PIN3                      (input pullup).
 * PB4  - PIN4                      (input pullup).
 * PB5  - PIN5                      (input pullup).
 * PB6  - I2C1_SCL                  (alternate 1).
 * PB7  - I2C1_SDA                  (alternate 1).
 * PB8  - GPIOB_CAN_RX              (alternate 4).
 * PB9  - GPIOB_CAN_TX              (alternate 4).
 * PB10 - PIN10                     (input pullup).
 * PB11 - PIN11                     (input pullup).
 * PB12 - PIN12                     (input pullup).
 * PB13 - PIN13                     (input pullup).
 * PB14 - PIN14                     (input pullup).
 * PB15 - PIN15                     (input pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_MPU_INT) |        \
                                     PIN_MODE_INPUT(GPIOB_NRF_INT) |        \
                                     PIN_MODE_OUTPUT(GPIOB_LED) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN5) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_RX) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_TX) |     \
                                     PIN_MODE_INPUT(GPIOB_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_MPU_INT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NRF_INT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_40M(GPIOB_MPU_INT) |        \
                                     PIN_OSPEED_40M(GPIOB_NRF_INT) |        \
                                     PIN_OSPEED_40M(GPIOB_LED) |            \
                                     PIN_OSPEED_40M(GPIOB_PIN3) |           \
                                     PIN_OSPEED_40M(GPIOB_PIN4) |           \
                                     PIN_OSPEED_40M(GPIOB_PIN5) |           \
                                     PIN_OSPEED_40M(GPIOB_I2C1_SCL) |       \
                                     PIN_OSPEED_40M(GPIOB_I2C1_SDA) |       \
                                     PIN_OSPEED_40M(GPIOB_CAN_RX) |         \
                                     PIN_OSPEED_40M(GPIOB_CAN_TX) |         \
                                     PIN_OSPEED_40M(GPIOB_PIN10) |          \
                                     PIN_OSPEED_40M(GPIOB_PIN11) |          \
                                     PIN_OSPEED_40M(GPIOB_PIN12) |          \
                                     PIN_OSPEED_40M(GPIOB_PIN13) |          \
                                     PIN_OSPEED_40M(GPIOB_PIN14) |          \
                                     PIN_OSPEED_40M(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_MPU_INT) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_NRF_INT) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_LED) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN3) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN4) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN5) |       \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C1_SCL) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C1_SDA) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_TX) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN10) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN11) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN12) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN13) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN14) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_MPU_INT) |           \
                                     PIN_ODR_LOW(GPIOB_NRF_INT) |           \
                                     PIN_ODR_LOW(GPIOB_LED) |               \
                                     PIN_ODR_LOW(GPIOB_PIN3) |              \
                                     PIN_ODR_LOW(GPIOB_PIN4) |              \
                                     PIN_ODR_LOW(GPIOB_PIN5) |              \
                                     PIN_ODR_LOW(GPIOB_I2C1_SCL) |          \
                                     PIN_ODR_LOW(GPIOB_I2C1_SDA) |          \
                                     PIN_ODR_LOW(GPIOB_CAN_RX) |            \
                                     PIN_ODR_LOW(GPIOB_CAN_TX) |            \
                                     PIN_ODR_LOW(GPIOB_PIN10) |             \
                                     PIN_ODR_LOW(GPIOB_PIN11) |             \
                                     PIN_ODR_LOW(GPIOB_PIN12) |             \
                                     PIN_ODR_LOW(GPIOB_PIN13) |             \
                                     PIN_ODR_LOW(GPIOB_PIN14) |             \
                                     PIN_ODR_LOW(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_MPU_INT, 0) |        \
                                     PIN_AFIO_AF(GPIOB_NRF_INT, 0) |        \
                                     PIN_AFIO_AF(GPIOB_LED, 0) |            \
                                     PIN_AFIO_AF(GPIOB_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOB_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL, 1) |       \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 1))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_CAN_RX, 4) |         \
                                     PIN_AFIO_AF(GPIOB_CAN_TX, 4) |         \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0))

/*
 * GPIOC setup:
 *
 * PC13 - PIN13                     (input pulldown).
 * PC14 - PIN14                     (input floating).
 * PC15 - PIN15                     (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_40M(GPIOC_PIN13) |          \
                                     PIN_OSPEED_40M(GPIOC_PIN14) |          \
                                     PIN_OSPEED_40M(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOC_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_PIN13) |             \
                                     PIN_ODR_LOW(GPIOC_PIN14) |             \
                                     PIN_ODR_LOW(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              0
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0))

/*
 * GPIOD setup:
 */
#define VAL_GPIOD_MODER             0
#define VAL_GPIOD_OTYPER            0
#define VAL_GPIOD_OSPEEDR           0
#define VAL_GPIOD_PUPDR             0
#define VAL_GPIOD_ODR               0
#define VAL_GPIOD_AFRL              0
#define VAL_GPIOD_AFRH              0

/*
 * GPIOF setup:
 *
 * PF0  - OSC_IN                    (input floating).
 * PF1  - OSC_OUT                   (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOF_OSC_OUT))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSC_OUT))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_2M(GPIOF_OSC_IN) |          \
                                     PIN_OSPEED_2M(GPIOF_OSC_OUT))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_OSC_OUT))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOF_OSC_OUT))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSC_IN, 0) |         \
                                     PIN_AFIO_AF(GPIOF_OSC_OUT, 0))
#define VAL_GPIOF_AFRH              0


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
