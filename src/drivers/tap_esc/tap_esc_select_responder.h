/*
 * tap_esc_select_responder.h
 *
 *  Created on: Apr 13, 2017
 *      Author: HaohuaChang
 */

#include <stdint.h>

/****************************************************************************
 * Name: select_responder
 *
 * Description:
 *   Select tap esc responder data for serial interface by 74hct151
 *
 ****************************************************************************/
namespace tap_esc
{
void select_responder(uint8_t sel);
} /* tap_esc */
