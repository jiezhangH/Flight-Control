/*
 * tap_74hct151.c
 *
 *  Created on: Apr 13, 2017
 *      Author: HaohuaChang
 */

#include <px4_config.h>

#include <stdbool.h>

#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

/****************************************************************************
 * Name: select_responder
 *
 * Description:
 *   Select tap esc responder data form ttyS2 by 74hct151
 *
 ****************************************************************************/

__EXPORT void select_responder(uint8_t sel)
{
#if defined(GPIO_S0)
	px4_arch_gpiowrite(GPIO_S0, sel & 1);
	px4_arch_gpiowrite(GPIO_S1, sel & 2);
	px4_arch_gpiowrite(GPIO_S2, sel & 4);
#endif
}
