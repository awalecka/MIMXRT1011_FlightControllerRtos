/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME "METRO-M7"

/*! @brief The USER_LED used for board */
#define LOGIC_LED_ON  (0U)
#define LOGIC_LED_OFF (1U)
#ifndef BOARD_USER_LED_GPIO
#define BOARD_USER_LED_GPIO GPIO1
#endif
#ifndef BOARD_USER_LED_GPIO_PIN
#define BOARD_USER_LED_GPIO_PIN (3U)
#endif

#define USER_LED_OFF() \
    GPIO_PortClear(BOARD_USER_LED_GPIO, 1U << BOARD_USER_LED_GPIO_PIN)                 /*!< Turn off target USER_LED */
#define USER_LED_ON() GPIO_PortSet(BOARD_USER_LED_GPIO, 1U << BOARD_USER_LED_GPIO_PIN) /*!<Turn on target USER_LED*/
#define USER_LED_TOGGLE()                                       \
    GPIO_PinWrite(BOARD_USER_LED_GPIO, BOARD_USER_LED_GPIO_PIN, \
                  0x1 ^ GPIO_PinRead(BOARD_USER_LED_GPIO, BOARD_USER_LED_GPIO_PIN)) /*!< Toggle target USER_LED */

/*! @brief The USER_LED used for board */
#define TIMING_GPIO_ON  (0U)
#define TIMING_GPIO_OFF (1U)
#ifndef BOARD_USER_TIMING_GPIO
#define BOARD_USER_TIMING_GPIO GPIO1
#endif
#ifndef BOARD_USER_TIMING_GPIO_PIN
#define BOARD_USER_TIMING_GPIO_PIN (4U)
#endif

#define USER_TIMING_OFF() \
    GPIO_PortClear(BOARD_USER_TIMING_GPIO, 1U << BOARD_USER_TIMING_GPIO_PIN)                 /*!< Turn off target USER_LED */
#define USER_TIMING_ON() GPIO_PortSet(BOARD_USER_TIMING_GPIO, 1U << BOARD_USER_TIMING_GPIO_PIN) /*!<Turn on target USER_LED*/
#define USER_TIMING_TOGGLE()                                       \
    GPIO_PinWrite(BOARD_USER_TIMING_GPIO, BOARD_USER_TIMING_GPIO_PIN, \
                  0x1 ^ GPIO_PinRead(BOARD_USER_TIMING_GPIO, BOARD_USER_TIMING_GPIO_PIN)) /*!< Toggle target USER_LED */

/*! @brief The flash size */
#define BOARD_FLASH_SIZE (0x800000U)

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
void BOARD_ConfigMPU(void);
#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
