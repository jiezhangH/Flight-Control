/*
 * tap_esc_select_responder.c
 *
 *  Created on: Apr 13, 2017
 *      Author: HaohuaChang
 */

#include <px4_config.h>

#include <stdbool.h>

#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>
#include "tap_esc_select_responder.h"

/****************************************************************************
 * Name: select_responder
 *
 * Description:
 *   Select tap esc responder for serial interface (device 74hct151).
 *   GPIOs to be defined in board_config.h
 *
 ****************************************************************************/

void select_responder(uint8_t sel)
{
#if defined(GPIO_S0)
	px4_arch_gpiowrite(GPIO_S0, sel & 1);
	px4_arch_gpiowrite(GPIO_S1, sel & 2);
	px4_arch_gpiowrite(GPIO_S2, sel & 4);
#endif
}
